import serial

arduino = None

def connect_device(comport, baud, time):
    global arduino
    arduino = serial.Serial(port=comport, baudrate=baud, timeout=time)

def write_data(to_write, log_file, time):
    global arduino
    arduino.write(bytes(to_write, 'utf-8'))
    log("Writing " + str(to_write) + " to serial", log_file, time)

def log(to_write, log_file, time):
    to_write = str(time) + ": " + str(to_write) + "\n"
    log_file.write(to_write)
    #print(to_write)

def read_data():
    serial_data = arduino.readline().rstrip()
    if(serial_data != b''):
        return str(serial_data)[2:-1]
    else:
        return ""

