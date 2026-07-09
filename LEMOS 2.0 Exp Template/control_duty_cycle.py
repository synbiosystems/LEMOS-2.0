"""
control_duty_cycle.py
--------------------------
Main code for the experiment. This is the fixed-duty-cycle variant.

This script drives an experiment where each well group receives light
on a FIXED green/red duty-cycle percentage (see the well-group lists
below, e.g. g5_ctrl_wells, g10_ctrl_wells, etc.) that stays constant
for the whole experiment - it does not adjust based on what the plate
reader measures.

At a high level, this script does three things in a loop, forever
(until you Ctrl+C):
 
  1. Drives the plate-reader software's GUI (via pyautogui screen
     clicks) to repeatedly run a measurement, export the data, and
     start again - see run_experiment() / _click_image().
 
  2. Computes, once per second-ish, what light signal each of the 32
     wells should currently be receiving (green "g", red "r", or off
     "o") based on how far we are into the current measurement cycle,
     and sends that as a single string over serial/BLE to the Arduino
     - see handle_timing().
 
  3. Watches (via the `watchdog` library, wired up in __main__) for the
     plate reader's CSV export to appear/change on disk, and when it
     does, parses it with datafile_manager, deletes the raw export, and
     restarts the measurement cycle - see process_datafile().
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
        
        # The wells below are grouped by what light "recipe" they should
        # receive. Each group is used in handle_timing() to decide when
        # a well switches from green ("g") to red ("r") light during the
        # "on" portion of each measurement cycle. Wells not in any of
        # these groups just get turned off ("o").
        self.neg_wells      =  ['A1', 'B1']
        self.r_ctrl_wells   =  ['A3','A4','A6']
        self.g_ctrl_wells   =  ['B3','B4','B6']
        self.g5_ctrl_wells  =  ['C1','D1','E1']
        self.g10_ctrl_wells =  ['F1','G1','H1']
        self.g15_ctrl_wells =  ['C3','C4','C6']
        self.g20_ctrl_wells =  ['D3','D4','D6']
        self.g40_ctrl_wells =  ['E3','E4','E6']
        self.g60_ctrl_wells =  ['F3','F4','F6']
        self.g80_ctrl_wells =  ['G3','G4','G6']
        self.d_ctrl_wells   =  ['H3','H4','H6']

        self.measurement_interval = 600
        self.default_on_time = (self.measurement_interval - 120) / 2
        self.max_on_time = self.measurement_interval - 120

        # --- State flags ---
        self.started = False                # whether an experiment cycle is currently running
        self.start_time = 0                 # time.time() value when the current cycle started
        self.need_retry = False             # set when the GUI automation failed to find "continue"
                                            # and needs a retry after export
        self.overall_start = time.time()    # Used as a t=0 reference for logging

        # Set up file system watcher
        logging.basicConfig(
            level=logging.INFO,
            format='%(asctime)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
    
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
        measurement_interval, builds a single string of one character
        per well (in ctrl_wells order), and sends it to the Arduino -
        but only if it's different from the last string sent, to avoid
        spamming redundant serial writes.
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
                if well in self.g_ctrl_wells:
                    to_send += "g"
                elif well in self.r_ctrl_wells:
                    to_send += "r"
                elif well in self.g5_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*5/100) else "r"
                elif well in self.g10_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*10/100) else "r"
                elif well in self.g15_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*15/100) else "r"
                elif well in self.g20_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*20/100) else "r"
                elif well in self.g40_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*40/100) else "r"
                elif well in self.g60_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*60/100) else "r"
                elif well in self.g80_ctrl_wells:
                    to_send += "g" if corrected_time < (30 + self.max_on_time*80/100) else "r"
                elif well in self.d_ctrl_wells:
                    to_send += "o"
                else:
                    to_send += "o"
            else:
                to_send += "o"

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
        export has appeared/changed on disk. Reads and archives the
        data via datafile_manager, deletes the raw export, and starts
        the next measurement cycle.
 
        Also handles the "need_retry" case: if the last run_experiment()
        call failed to find the "Continue" button (see run_experiment),
        this function keeps retrying run_experiment() once the datafile
        shows up, since that was likely evidence the export happened
        anyway.
        """
        if not os.path.exists(self.datafile_name):
            # print("File doesn't exist yet, skipping processing")
            if self.need_retry:
                # print("Will retry experiment...")
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
                # Loop roughly twice a second - frequent enough to catch
                # timing-window transitions without hammering the CPU
                # or the serial port.
                time.sleep(0.5)
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