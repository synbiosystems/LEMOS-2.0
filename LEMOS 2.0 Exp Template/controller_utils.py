"""
controller_utils.py
--------------------
Small support classes/functions shared by the main experiment controller
 
    - TeeLogger      : mirrors everything printed to the console into a
                        log file.
    - FileHandler    : monitors a folder for changes to the plate-reader's
                        CSV export and tells the controller to process it 
                        when it changes.
    - compute_integral / compute_derivative : generic helper math for a
                        PID-style controller (integral and derivative of
                        an error signal over time).
"""

import sys, time, os
import watchdog.events

# ------------------------------------------------------------
# EVENT LOGGER
# ------------------------------------------------------------
class TeeLogger:
    """
    Redirects stdout so every print() goes to BOTH the real terminal
    and a log file on disk. Used in ble_control_duty_cycle.py via:
        sys.stdout = TeeLogger(f"{self.filename}.txt")
        sys.stderr = sys.stdout
    so both normal output and errors get captured.
    """
    def __init__(self, filename):
        self.terminal = sys.__stdout__
        self.log = open(filename, "a")

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        self.terminal.flush()
        self.log.flush()

# ------------------------------------------------------------
# FILE HANDLER
# ------------------------------------------------------------
class FileHandler(watchdog.events.PatternMatchingEventHandler):
    """
    A watchdog event handler that watches for the plate-reader's CSV
    export file to be created/modified, and triggers the controller's
    datafile-processing step when that happens.
    """
    
    def __init__(self, filename, controller):
        super().__init__(patterns=[filename])
        self.controller = controller
        self.processing = False

    def on_modified(self, event):
        if self.processing:
            return
        
        if not os.path.exists(event.src_path):
            return
        
        print("Datafile was modified! - % s" % event.src_path)
        self.processing = True
        # Small delay to give the plate-reader software time to finish
        # writing the file completely before we try to read it.
        time.sleep(3)
        self.controller.process_datafile()
        self.processing = False

# ------------------------------------------------------------
# INTEGRAL / DERIVATIVE
# ------------------------------------------------------------
def compute_integral(pid_times_stamp, errors):
    """
    Computes full trapezoidal integral of error history from time = 0.
    `errors` is the list of error values for a single well.
    """
    if len(pid_times_stamp) < 2 or len(errors) < 2:
        return 0.0
    integral = 0.0
    for i in range(1, len(pid_times_stamp)):
        dt = pid_times_stamp[i] - pid_times_stamp[i-1]
        integral += 0.5 * (errors[i] + errors[i-1]) * dt
    return integral


def compute_derivative(pid_times_stamp, errors):
    """
    Computes derivative of the error (finite difference).
    `errors` is the list of error values for a single well.
    """
    if len(pid_times_stamp) < 2 or len(errors) < 2:
        return 0.0
    dt = pid_times_stamp[-1] - pid_times_stamp[-2]
    if dt == 0:
        return 0.0
    return (errors[-1] - errors[-2]) / dt