import sys, logging, pyautogui, time, os
import numpy as np, pandas as pd
from watchdog.observers import Observer
import datafile_manager, ble_comms
from controller_utils import TeeLogger, FileHandler, compute_integral, compute_derivative

pyautogui.FAILSAFE = False

# ------------------------------------------------------------
# CONTROLLER
# ------------------------------------------------------------
class ExperimentController:
    def __init__(self, file, file_path):
        self.last_sent_signal = None
        self.filename = file
        self.datafile_path = file_path
        self.datafile_name = os.path.join(file_path, f"{file}.csv")

        # Logging
        sys.stdout = TeeLogger(f"{self.filename}.txt")
        sys.stderr = sys.stdout

        # BLE Communication Init
        ble_comms.connect_device("COM7", 115200, 0.1)
        self.log_file = open(f"{self.filename}_b.out", "a")

        # Wells
        self.ctrl_wells = [
            'A1','B1','C1','D1','E1','F1','G1','H1',
            'H3','G3','F3','E3','D3','C3','B3','A3',
            'A4','B4','C4','D4','E4','F4','G4','H4',
            'H6','G6','F6','E6','D6','C6','B6','A6'
        ]
        self.neg_wells =  ['A1', 'B1']
        self.r        =   ['A3','A4','A6']
        self.g        =   ['B3','B4','B6']
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
        #                 K_pid    tau_I_pid (s)   tau_D_pid (s)
        self.pid_gains = {
            1: (0.020, 1500 * 60, 100 * 60),   # gain group 1
            2: (0.020, 1500 * 60, 120 * 60),   # gain group 2
            3: (0.015, 1500 * 60, 100 * 60),   # gain group 3
            4: (0.015, 1500 * 60, 120 * 60),   # gain group 4
        }

        # Target setpoints, one per setpoint group (1-2).
        self.setpoints = {
            1: 11500,   # setpoint group 1
            2: 18500,   # setpoint group 2
        }

        # Per-well lookups so PID math doesn't need to know which group a well is in
        self.well_gains = {}
        self.well_setpoint = {}
        for group, gain_num, sp_num in self.pid_well_groups:
            for well in group:
                self.well_gains[well] = self.pid_gains[gain_num]
                self.well_setpoint[well] = self.setpoints[sp_num]

        self.errors = {w: [] for w in self.ctrl_wells}
        self.pid_times_stamp = []
        self.pid_times = [self.default_on_time] * len(self.ctrl_wells)

        self.error_df = pd.DataFrame(columns=["time"] + self.pid)

        self.started = False
        self.start_time = 0
        self.need_retry = False

        # Reload error history if it exists (handles restarts)
        error_csv = 'Datafile/errors.csv'
        if os.path.exists(error_csv):
            self._reload_error_history(error_csv)

        # Set overall_start: continue from last saved timestamp or start fresh
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
        try:
            df = pd.read_csv(path)
            df = df[df['time'] < 1e8]
            self.error_df = df.copy()

            for _, row in df.iterrows():
                self.pid_times_stamp.append(row['time'])
                for well in self.pid:
                    if well in row and pd.notna(row[well]):
                        self.errors[well].append(row[well])

            # Recompute ctrl_times from reloaded history
            for well in self.pid:
                if len(self.errors[well]) == 0:
                    continue

                error = self.errors[well][-1]
                K_pid, tau_I_pid, tau_D_pid = self.well_gains[well]

                if len(self.errors[well]) >= 2:
                    integral = compute_integral(self.pid_times_stamp, self.errors[well])
                    deriv = compute_derivative(self.pid_times_stamp, self.errors[well])
                    on = K_pid * (error + tau_D_pid * deriv + (1/tau_I_pid) * integral)
                else:
                    on = K_pid * error  
                    
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
        for _ in range(retries):
            try:
                loc = pyautogui.center(pyautogui.locateOnScreen(f"search_targets/{name}.PNG"))
                pyautogui.click(loc)
                time.sleep(2)
                return True
            except Exception as e:
                print(f"  Waiting for {name}: {e}")
                time.sleep(wait)
        print("Could not find", name)
        return False

    # ------------------------------------------------------------
    # TIMING
    # ------------------------------------------------------------
    def handle_timing(self):
        if not self.started:
            return

        curr_time = time.time() - self.start_time
        corrected_time = curr_time % self.measurement_interval
        to_send = ""

        for well in self.ctrl_wells:
            if corrected_time > 30 and corrected_time < (self.measurement_interval - 90):
                if well in self.g:
                    to_send += "g"
                elif well in self.r:
                    to_send += "r"
                elif well in self.pid:
                    idx = self.ctrl_wells.index(well)
                    to_send += "g" if corrected_time < (30 + self.pid_times[idx]) else "r"
                else:
                    to_send += "o"
            else:
                to_send += "o"

        if to_send != self.last_sent_signal:
            self.last_sent_signal = to_send
            ble_comms.write_data(to_send + "\n", self.log_file, time.time() - self.overall_start)

    # ------------------------------------------------------------
    # PROCESS DATAFILE
    # ------------------------------------------------------------
    def process_datafile(self):
        if not os.path.exists(self.datafile_name):
            if self.need_retry:
                time.sleep(2)
                self.run_experiment()
            return

        try:
            datafile_manager.read_and_save(self.datafile_name)
        except BaseException as e:
            print("Error reading csv export:", e)
            if self.need_retry:
                print("Error occurred during retry, will try again...")
                time.sleep(2)
                self.run_experiment()
            return

        fl_latest = datafile_manager.get_fl_latest()
        od_latest = datafile_manager.get_od_latest()

        #negative control calculation
        newest_idx = list(fl_latest[self.neg_wells[0]].keys())[-1]
        neg_fls = []
        neg_ods = []

        for well in self.neg_wells:
            fl_val = int(fl_latest[well][newest_idx])
            od_val = min(float(od_latest[well][newest_idx]), 3)
            neg_fls.append(fl_val)
            neg_ods.append(od_val)

        neg_fl = np.mean(neg_fls)
        neg_od = np.mean(neg_ods)

        # Compute candidate PID-well errors WITHOUT committing them yet, so it can compare against the last saved row before deciding whether to append.
        fl_by_od_cache = {}
        for well in self.pid:
            od = od_latest[well][newest_idx]
            od = 1 if str(od).upper() == "OVRFLW" else min(float(od), 3)
            fl = fl_latest[well][newest_idx]
            fl = 100000 if str(fl).upper() == "OVRFLW" else int(fl)
            fl_by_od_cache[well] = (fl / od) - (neg_fl / neg_od)

        candidate_errors = {
            well: self.well_setpoint[well] - fl_by_od_cache[well] for well in self.pid
        }

        # Compare against the last recorded row, ignoring timestamp on purpose.
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

        curr_time = time.time() - self.overall_start
        self.pid_times_stamp.append(curr_time)

        for i, well in enumerate(self.ctrl_wells):
            on = self.default_on_time

            if well in self.pid:
                # PID Control Wells
                error = candidate_errors[well]
                self.errors[well].append(error)
                K_pid, tau_I_pid, tau_D_pid = self.well_gains[well]

                if len(self.pid_times_stamp) >= 2:
                    integral = compute_integral(self.pid_times_stamp, self.errors[well])
                    deriv = compute_derivative(self.pid_times_stamp, self.errors[well])
                    on = K_pid * (error + tau_D_pid*deriv + (1/tau_I_pid)*integral)
                else:
                    on = K_pid * error # Not enough data points yet

            on_time = max(0, min(self.max_on_time, on))
            # print("calculated raw on_time of:", on_time, "seconds")
            self.pid_times[i] = on_time
            # print()
        # print("New duration setpoints calculated: ", self.pid_times)

        # Save errors to CSV
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
        try:
            self.run_experiment()
            while True:
                self.handle_timing()
                msg = ble_comms.read_data()
                if msg != "":
                    print(msg)
                time.sleep(0.5)
        except KeyboardInterrupt:
            print("Shutting down experiment.")
            self.log_file.close()

# ------------------------------------------------------------
# MAIN
# ------------------------------------------------------------
if __name__ == "__main__":
    file = "Template"
    base_path = sys.argv[1] if len(sys.argv) > 1 else "."
    file_path = os.path.join(base_path, "Datafile")

    controller = ExperimentController(file, file_path)

    datafile_event_handler = FileHandler(f"{file}.csv", controller)
    datafile_observer = Observer()
    datafile_observer.schedule(datafile_event_handler, file_path, recursive=True)
    datafile_observer.start()

    controller.run()