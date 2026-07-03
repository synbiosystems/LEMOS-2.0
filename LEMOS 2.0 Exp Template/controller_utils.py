import sys, time, os
import watchdog.events

# EVENT LOGGER
class TeeLogger:
    def __init__(self, filename):
        self.terminal = sys.__stdout__
        self.log = open(filename, "a")

    def write(self, message):
        self.terminal.write(message)
        self.log.write(message)

    def flush(self):
        self.terminal.flush()
        self.log.flush()

# FILE HANDLER
class FileHandler(watchdog.events.PatternMatchingEventHandler):
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
        time.sleep(3)
        self.controller.process_datafile()
        self.processing = False

# INTEGRAL / DERIVATIVE
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