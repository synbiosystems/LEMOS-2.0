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

        self.started = False
        self.start_time = 0
        self.need_retry = False
        self.overall_start = time.time()

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

        if to_send != self.last_sent_signal:
            self.last_sent_signal = to_send
            ble_comms.write_data(to_send + "\n", self.log_file, time.time() - self.overall_start)

    # ------------------------------------------------------------
    # PROCESS DATAFILE
    # ------------------------------------------------------------
    def process_datafile(self):
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