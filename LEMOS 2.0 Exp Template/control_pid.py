"""
control_pid.py
------------------------
Main code for the experiment. This is the PID variant.

This script drives an experiment where each PID-controlled well's "on
time" (how long it stays green before switching to red) is adjusted
every measurement cycle by a PID controller, based on how far that
well's measured fluorescence/OD ratio is from a target setpoint.

At a high level, this script does three things in a loop, forever
(until you Ctrl+C):

  1. Drives the plate-reader software's GUI (via pyautogui screen
     clicks) to repeatedly run a measurement, export the data, and
     start again - see run_experiment() / _click_image().

  2. Computes, once per second-ish, what light signal each of the 32
     wells should currently be receiving (green "g", red "r", or off
     "o") based on how far we are into the current measurement cycle
     AND (for PID wells) the current PID-calculated on-time for that
     well - see handle_timing().

  3. Watches (via the `watchdog` library, wired up in __main__) for the
     plate reader's CSV export to appear/change on disk, and when it
     does: parses it with datafile_manager, computes each PID well's
     error vs its setpoint, runs the PID math to get a new on-time per
     well, logs the errors to Datafile/errors.csv, deletes the raw
     export, and restarts the measurement cycle - see process_datafile().
"""

import sys, logging, pyautogui, time, os
import numpy as np, pandas as pd
from watchdog.observers import Observer
import datafile_manager, ble_comms
from controller_utils import TeeLogger, FileHandler, compute_integral, compute_derivative

# Disables pyautogui's "fling mouse to a screen corner to abort" safety
# feature. Left on (True) this would let you emergency-stop the script
# by yanking the mouse to a corner; it's turned off here so accidental
# mouse movement during the automated clicking doesn't kill the run.
pyautogui.FAILSAFE = False

# ------------------------------------------------------------
# CONTROLLER
# ------------------------------------------------------------
class ExperimentController:
    def __init__(self, file, file_path):
        """
        Parameters
        ----------
        file : str
            Base name (no extension) used for the datafile, the log
            files, and the .csv the plate reader is expected to export.
            e.g. "Template" -> looks for "Template.csv".
        file_path : str
            Folder where the plate reader will drop its CSV export
            (passed in as the "Datafile" subfolder of base_path - see
            __main__ below).
        """
        self.last_sent_signal = None
        self.filename = file
        self.datafile_path = file_path
        self.datafile_name = os.path.join(file_path, f"{file}.csv")

        # --- Logging setup ---
        sys.stdout = TeeLogger(f"{self.filename}.txt")
        sys.stderr = sys.stdout

        # --- BLE / Serial Communication Init ---
        # NOTE: "COM7" is the Windows serial port for the Arduino - this
        # is hardware/computer specific and is the most likely thing to
        # need changing if this is run on a different machine or the
        # USB cable is plugged into a different port.
        ble_comms.connect_device("COM7", 115200, 0.1)
        self.log_file = open(f"{self.filename}_b.out", "a")

        # --- Well plate layout ---
        self.ctrl_wells = [
            'A1','B1','C1','D1','E1','F1','G1','H1',
            'H3','G3','F3','E3','D3','C3','B3','A3',
            'A4','B4','C4','D4','E4','F4','G4','H4',
            'H6','G6','F6','E6','D6','C6','B6','A6'
        ]
        self.neg_wells =  ['A1', 'B1']
        self.r        =   ['A3','A4','A6']
        self.g        =   ['B3','B4','B6']
        
        # The 8 groups below are each controlled by their own PID loop.
        # Naming convention: pid<gain_group>_sp<setpoint_group>, e.g.
        # "pid1_sp1" = wells using gain-group 1's (K, tau_I, tau_D) and
        # targeting setpoint-group 1's value. This lets you test
        # multiple gain tunings against multiple setpoints
        # simultaneously in a single plate.
        self.pid1_sp1 =   ['C1','D1','E1']
        self.pid2_sp1 =   ['F1','G1','H1']
        self.pid3_sp1 =   ['C3','C4','C6']
        self.pid4_sp1 =   ['D3','D4','D6']
        self.pid1_sp2 =   ['E3','E4','E6']
        self.pid2_sp2 =   ['F3','F4','F6']
        self.pid3_sp2 =   ['G3','G4','G6']
        self.pid4_sp2 =   ['H3','H4','H6']

        # Each of the 8 well-groups is (gain_group_number, setpoint_group_number).
        # gain_group selects which (K, tau_I, tau_D) triple to use (1-4, see self.pid_gains).
        # setpoint_group selects which target value to use (1 or 2, see self.setpoints).
        self.pid_well_groups = [
            (self.pid1_sp1, 1, 1),
            (self.pid2_sp1, 2, 1),
            (self.pid3_sp1, 3, 1),
            (self.pid4_sp1, 4, 1),
            (self.pid1_sp2, 1, 2),
            (self.pid2_sp2, 2, 2),
            (self.pid3_sp2, 3, 2),
            (self.pid4_sp2, 4, 2),
        ]

        # Flattened list of every well under PID control (used for generic iteration)
        self.pid = [w for group, _, _ in self.pid_well_groups for w in group]

        self.measurement_interval = 600
        self.default_on_time = (self.measurement_interval - 120) / 2
        self.max_on_time = self.measurement_interval - 120

        # PID gains, one (K, tau_I, tau_D) triple per gain group (1-4).
        # Edit these four rows to test different control strategies per group.
        # Each gain group is shared by both setpoint groups (sp1 and sp2).
        #   K_pid      = proportional gain
        #   tau_I_pid  = integral time constant, in seconds
        #   tau_D_pid  = derivative time constant, in seconds 
        #                 K_pid    tau_I_pid (s)   tau_D_pid (s)
        self.pid_gains = {
            1: (0.020, 1500 * 60, 100 * 60),   # gain group 1
            2: (0.020, 1500 * 60, 120 * 60),   # gain group 2
            3: (0.015, 1500 * 60, 100 * 60),   # gain group 3
            4: (0.015, 1500 * 60, 120 * 60),   # gain group 4
        }

        # Target setpoints, one per setpoint group (1-2). These are the
        # target (fluorescence / OD) ratio values (baseline-corrected
        # against the negative control wells) that the PID loop tries to
        # drive each well's measured value toward, by adjusting how long
        # that well's light stays green each cycle.
        self.setpoints = {
            1: 11500,   # setpoint group 1
            2: 18500,   # setpoint group 2
        }

        # Per-well lookups so PID math doesn't need to know which group a well is in.
        # Built once here from pid_well_groups/pid_gains/setpoints above, so the
        # rest of the code can just do self.well_gains[well] / self.well_setpoint[well]
        # without caring about gain-group/setpoint-group numbers.
        self.well_gains = {}
        self.well_setpoint = {}
        for group, gain_num, sp_num in self.pid_well_groups:
            for well in group:
                self.well_gains[well] = self.pid_gains[gain_num]
                self.well_setpoint[well] = self.setpoints[sp_num]

        # self.errors: per-well history of error values (setpoint - measured),
        # one list per PID well, appended to every time process_datafile() runs.
        # Used by compute_integral()/compute_derivative() to get the I and D terms.
        self.errors = {w: [] for w in self.ctrl_wells}
        
        # Timestamps (seconds since overall_start) corresponding to each
        # entry appended to self.errors - i.e. one timestamp per measurement
        # cycle, shared across all wells.
        self.pid_times_stamp = []
        
        # Current "on time" (seconds of green light) for each well, indexed
        # the same way as self.ctrl_wells. Starts at default_on_time for
        # every well and gets updated by process_datafile() each cycle
        # once real error data is available. This is what handle_timing()
        # actually reads from when deciding whether to send "g" or "r".
        self.pid_times = [self.default_on_time] * len(self.ctrl_wells)

        # Running table of error values over time, one column per PID well
        # plus a "time" column - this is what gets written to/read from
        # Datafile/errors.csv (see _reload_error_history() and
        # process_datafile()).
        self.error_df = pd.DataFrame(columns=["time"] + self.pid)

        self.started = False
        self.start_time = 0
        self.need_retry = False

        # Reload error history if it exists (handles restarts).
        # This lets the script pick up where it left off (PID on-times and
        # error history intact) if it's stopped and restarted mid-experiment,
        # instead of starting the PID loop from scratch.
        error_csv = 'Datafile/errors.csv'
        if os.path.exists(error_csv):
            self._reload_error_history(error_csv)

        # Set overall_start: continue from last saved timestamp or start fresh.
        # If we reloaded history above, overall_start is backdated so that
        # time.time() - overall_start lines up with the timestamps already
        # in pid_times_stamp/error_df, keeping the elapsed-time axis continuous
        # across a restart. If there's no history, it's just "now".
        if self.pid_times_stamp:
            self.overall_start = time.time() - self.pid_times_stamp[-1]
        else:
            self.overall_start = time.time()

        # Set up file system watcher
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )

    def _reload_error_history(self, path):
        """
        Restore self.errors, self.pid_times_stamp, self.error_df, and
        self.pid_times from a previously-saved Datafile/errors.csv, so
        that restarting the script mid-experiment doesn't lose PID
        history or reset every well's on-time back to default_on_time.
        """
        try:
            df = pd.read_csv(path)
            # Sanity filter: drop any rows with an implausibly large time
            # value (>= 1e8 seconds, i.e. ~3+ years) - guards against a
            # rubbish timestamp fouling the reloaded history.
            df = df[df['time'] < 1e8]
            self.error_df = df.copy()

            # Rebuild the in-memory error history from the saved rows.
            for _, row in df.iterrows():
                self.pid_times_stamp.append(row['time'])
                for well in self.pid:
                    if well in row and pd.notna(row[well]):
                        self.errors[well].append(row[well])

            # Recompute each PID well's current on-time from its reloaded
            # error history.
            for well in self.pid:
                if len(self.errors[well]) == 0:
                    continue

                error = self.errors[well][-1]
                K_pid, tau_I_pid, tau_D_pid = self.well_gains[well]

                if len(self.errors[well]) >= 2:
                    # Full PID: proportional + integral + derivative terms.
                    integral = compute_integral(self.pid_times_stamp, self.errors[well])
                    deriv = compute_derivative(self.pid_times_stamp, self.errors[well])
                    on = K_pid * (error + tau_D_pid * deriv + (1/tau_I_pid) * integral)
                else:
                    # Only one data point so far - not enough history for
                    # integral/derivative terms, so fall back to proportional-only.
                    on = K_pid * error  

                # Clamp the computed on-time to a valid range: never negative,
                # never more than the physically available on-window.
                on_time = max(0, min(self.max_on_time, on))
                idx = self.ctrl_wells.index(well)
                self.pid_times[idx] = on_time
                print(f"Reloaded ctrl_time for {well}: {on_time:.1f}s")
                    
            print(f"Reloaded {len(self.pid_times_stamp)} historical error points")
        except Exception as e:
            print(f"Could not reload error history: {e}")

    # ------------------------------------------------------------
    # GUI
    # ------------------------------------------------------------
    def run_experiment(self):
        """
        Automates the plate-reader software's GUI to start a new
        measurement run: clicks "Run", then "Continue". If "Continue"
        can't be found (e.g. a dialog didn't appear as expected), it
        falls back to clicking "Export" and flags need_retry so the
        run will be retried once the datafile arrives.

        The button images it looks for live in "search_targets/" as
        "1.run.PNG", "2.continue.PNG", "3.export.PNG" - these need to
        match what's actually on screen for the plate-reader software
        used, at whatever screen resolution/scaling this is run at.
        """
        print("Starting in 2 seconds...")
        time.sleep(2)
        self._click_image("1.run", retries=3)

        # Try to click continue, retry run if continue not found
        if self._click_image("2.continue", retries=3):
            self.need_retry = False
        else:
            print("Continue button not found, will retry after export...")
            self.need_retry = True
            time.sleep(20)
            self._click_image("3.export", retries=3)
            return

        self.start_time = time.time()
        self.started = True

    def _click_image(self, name, retries=3, wait=5):
        """
        Repeatedly search the screen for an image (a button screenshot
        stored under search_targets/<name>.PNG) and click its center
        when found.
        """
        for _ in range(retries):
            try:
                loc = pyautogui.center(pyautogui.locateOnScreen(f"search_targets/{name}.PNG"))
                pyautogui.click(loc)
                time.sleep(2)
                return True
            except Exception as e:
                # locateOnScreen raises an exception if the image isn't found on screen
                print(f"  Waiting for {name}: {e}")
                time.sleep(wait)
        print("Could not find", name)
        return False

    # ------------------------------------------------------------
    # TIMING
    # ------------------------------------------------------------
    def handle_timing(self):
        """
        Called repeatedly (every 0.5s, from run()) while an experiment
        is active. Figures out what light color/state each well should
        currently be showing based on elapsed time within the current
        measurement_interval - using each PID well's current on-time
        from self.pid_times instead of a fixed percentage - builds a
        single string of one character per well (in ctrl_wells order),
        and sends it to the Arduino, but only if it's different from
        the last string sent, to avoid spamming redundant serial writes.
        """
        if not self.started:
            return

        # Elapsed time since this cycle began.
        curr_time = time.time() - self.start_time
        # Position within the current 600s cycle
        corrected_time = curr_time % self.measurement_interval
        to_send = ""

        for well in self.ctrl_wells:
            # Lights are only ever "on" (green/red) during the window
            # from 30s to (measurement_interval - 90)s within each
            # cycle. Outside that window (the first 30s and last 90s)
            # every well is forced off - this is the buffer time around
            # the actual plate-reader measurement so light doesn't
            # interfere with it.
            if corrected_time > 30 and corrected_time < (self.measurement_interval - 90):
                if well in self.g:
                    to_send += "g"
                elif well in self.r:
                    to_send += "r"
                elif well in self.pid:
                    idx = self.ctrl_wells.index(well)
                    to_send += "g" if corrected_time < (30 + self.pid_times[idx]) else "r"
                else:
                    to_send += "o"   # any well not explicitly grouped defaults to off
            else:
                to_send += "o"       # outside the on-window: everything off

        # Only send a new command over serial if the signal has actually
        # changed since last time, to avoid flooding the Arduino with
        # identical repeat commands every 0.5s.
        if to_send != self.last_sent_signal:
            self.last_sent_signal = to_send
            ble_comms.write_data(to_send + "\n", self.log_file, time.time() - self.overall_start)

    # ------------------------------------------------------------
    # PROCESS DATAFILE
    # ------------------------------------------------------------
    def process_datafile(self):
        """
        Called when the watchdog file-watcher (see FileHandler in
        controller_utils.py) detects that the plate reader's CSV
        export has appeared/changed on disk.

          1. Reads the new datafile.
          2. Computes a negative-control baseline from self.neg_wells.
          3. Computes each PID well's baseline-corrected fl/od ratio
             and its error vs that well's setpoint.
          4. Checks whether this is a genuinely new measurement (guards
             against processing the same export twice - see the
             "is_duplicate" check below).
          5. Runs the PID math (P, I, D terms) to get a new on-time per
             well, clamped to a valid range.
          6. Appends the new error row to Datafile/errors.csv.
          7. Deletes the raw export and restarts the measurement cycle.

        Also handles the "need_retry" case: if the last run_experiment()
        call failed to find the "Continue" button (see run_experiment),
        this function keeps retrying run_experiment() once the datafile
        shows up, since that was likely evidence the export happened
        anyway.
        """
        if not os.path.exists(self.datafile_name):
            if self.need_retry:
                time.sleep(2)
                self.run_experiment()
            return

        try:
            datafile_manager.read_and_save(self.datafile_name)
        except BaseException as e:
            # Broad exception here is intentional: any failure while
            # parsing (bad/partial CSV, unexpected format, etc.) should
            # not crash the whole controller - just log it and, if we
            # were in a retry situation, try running again.
            print("Error reading csv export:", e)
            if self.need_retry:
                print("Error occurred during retry, will try again...")
                time.sleep(2)
                self.run_experiment()
            return

        fl_latest = datafile_manager.get_fl_latest()
        od_latest = datafile_manager.get_od_latest()

        #negative control calculation
        # fl_latest/od_latest are dicts of {well: {row_index: value}} - grab
        # the most recent row index (the last key) so we always read the
        # newest measurement, regardless of how many rows are in the table.
        newest_idx = list(fl_latest[self.neg_wells[0]].keys())[-1]
        neg_fls = []
        neg_ods = []

        for well in self.neg_wells:
            fl_val = int(fl_latest[well][newest_idx])
            # OD readings are capped at 3 - the plate reader's sensor
            # saturates/becomes unreliable above this, so treat anything
            # higher as just "3" to avoid the ratio below blowing up.
            od_val = min(float(od_latest[well][newest_idx]), 3)
            neg_fls.append(fl_val)
            neg_ods.append(od_val)

        # Average fluorescence and OD across the negative control wells -
        # this baseline gets subtracted from every PID well's fl/od ratio
        # below, so the PID error reflects signal ABOVE background rather
        # than the raw (background-inflated) reading.
        neg_fl = np.mean(neg_fls)
        neg_od = np.mean(neg_ods)

        # Compute candidate PID-well errors WITHOUT committing them yet, so it can compare against the last saved row before deciding whether to append.
        fl_by_od_cache = {}
        for well in self.pid:
            od = od_latest[well][newest_idx]
            od = 1 if str(od).upper() == "OVRFLW" else min(float(od), 3)
            fl = fl_latest[well][newest_idx]
            fl = 100000 if str(fl).upper() == "OVRFLW" else int(fl)
            # Baseline-corrected fluorescence-per-OD signal for this well:
            # (well's fl/od) minus (negative control's fl/od).
            fl_by_od_cache[well] = (fl / od) - (neg_fl / neg_od)

        # error = how far below/above target this well currently is.
        candidate_errors = {
            well: self.well_setpoint[well] - fl_by_od_cache[well] for well in self.pid
        }

        # Compare against the last recorded row, ignoring timestamp on purpose.
        # Guards against the same plate-reader export being processed twice
        # (e.g. if the file-watcher fires more than once for one export, or
        # the reader re-exports identical data) - if every well's computed
        # error exactly matches the last saved row, treat this as a
        # duplicate and skip appending/re-running the PID math for it.
        if len(self.error_df) > 0:
            last_row = self.error_df.iloc[-1]
            is_duplicate = True
            for well in self.pid:
                last_val = last_row.get(well, None)
                if last_val is None or last_val == "" or pd.isna(last_val):
                    is_duplicate = False
                    break
                if not np.isclose(float(last_val), candidate_errors[well], rtol=1e-9, atol=1e-9):
                    is_duplicate = False
                    break

            if is_duplicate:
                print("Computed error values match the last recorded measurement, skipping append...")
                try:
                    datafile_manager.remove()
                except BaseException as e:
                    print("Error deleting csv export...", e)

                print("Restarting experiment after processing datafile...")
                time.sleep(2)
                self.run_experiment()
                return

        # Not a duplicate - this is a genuinely new measurement, so record
        # its timestamp and proceed to update the PID state for every well.
        curr_time = time.time() - self.overall_start
        self.pid_times_stamp.append(curr_time)

        for i, well in enumerate(self.ctrl_wells):
            on = self.default_on_time

            if well in self.pid:
                # --- PID Control Wells ---
                error = candidate_errors[well]
                self.errors[well].append(error)
                K_pid, tau_I_pid, tau_D_pid = self.well_gains[well]

                if len(self.pid_times_stamp) >= 2:
                    # Enough history to compute integral (accumulated past
                    # error) and derivative (rate of change of error) terms,
                    # so use the full PID formula:
                    #   on = K * (error + tau_D * d(error)/dt + (1/tau_I) * integral(error))
                    integral = compute_integral(self.pid_times_stamp, self.errors[well])
                    deriv = compute_derivative(self.pid_times_stamp, self.errors[well])
                    on = K_pid * (error + tau_D_pid*deriv + (1/tau_I_pid)*integral)
                else:
                    # Not enough data points yet for I/D terms - proportional-only.
                    on = K_pid * error # Not enough data points yet

            # Clamp to a valid on-time: never negative, never longer than
            # the on-window actually allows.
            on_time = max(0, min(self.max_on_time, on))
            # print("calculated raw on_time of:", on_time, "seconds")
            self.pid_times[i] = on_time
            # print()
        # print("New duration setpoints calculated: ", self.pid_times)

        # Save errors to CSV
        # Build one row per measurement cycle: timestamp + each PID well's
        # latest error value, so Datafile/errors.csv keeps a full history
        # that can be reloaded on restart (_reload_error_history) or
        # inspected/plotted afterward.
        row = {"time": curr_time}
        # Safety check to prevent index error in case pid is assigned to a non-existing well
        for well in self.pid:
            if len(self.errors[well]) > 0:
                row[well] = self.errors[well][-1]
            else:
                row[well] = ""
                
        new_row = pd.DataFrame([row])
        self.error_df = pd.concat([self.error_df, new_row], ignore_index=True)
        self.error_df.to_csv('Datafile/errors.csv', index=False)

        try:
            datafile_manager.remove()
        except BaseException as e:
            print("Error deleting csv export...", e)

        print("Restarting experiment after processing datafile...")
        time.sleep(2)
        self.run_experiment()

    # ------------------------------------------------------------
    # RUN LOOP
    # ------------------------------------------------------------
    def run(self):
        """
        Main blocking loop: starts the first experiment cycle, then
        repeatedly updates the light-signal timing and checks for any
        incoming serial messages from the Arduino (e.g. status/debug
        output), until interrupted with Ctrl+C.
        """
        try:
            self.run_experiment()
            while True:
                self.handle_timing()
                msg = ble_comms.read_data()
                if msg != "":
                    print(msg)
                time.sleep(0.5)
                # Loop roughly twice a second - frequent enough to catch
                # timing-window transitions without hammering the CPU
                # or the serial port.
        except KeyboardInterrupt:
            print("Shutting down experiment.")
            self.log_file.close()

# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
if __name__ == "__main__":
    # Base name for the expected datafile / log files. Change this if
    # the plate-reader software is configured to export under a
    # different name.
    file = "Template"
    # Optional command-line argument: the root folder to look for/watch
    # the "Datafile" subfolder in. Defaults to the current directory if
    # not provided
    base_path = sys.argv[1] if len(sys.argv) > 1 else "."
    file_path = os.path.join(base_path, "Datafile")

    controller = ExperimentController(file, file_path)

    # Set up a filesystem watcher that calls controller.process_datafile()
    # whenever "Template.csv" is created/modified inside file_path (and
    # its subfolders, since recursive=True).
    datafile_event_handler = FileHandler(f"{file}.csv", controller)
    datafile_observer = Observer()
    datafile_observer.schedule(datafile_event_handler, file_path, recursive=True)
    datafile_observer.start()
    
    # Enter the main timing/serial loop (blocks until Ctrl+C).
    controller.run()