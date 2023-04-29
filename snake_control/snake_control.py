from pynput import keyboard
from pynput.keyboard import Key, Listener, KeyCode
import time
import os
import serial
# import threading
# import csv

# from read_keyboard import rkb as rkb

# rkb = rkb()
val_w, val_a, val_s, val_d = 0,0,0,0
# user velocity pre-define
vel_user = 0
# restart counter
restart_counter = 0

# initialize serial communication for arduino
# ser1 = serial.Serial("/dev/ttyACM1", 9600, timeout=0.3)  # snake COM port
ser = serial.Serial('/dev/ttyACM1', 115200)
print("Reset Arduino")
print("(A) Left, (D) Right, (R) Automatic moving, (S): Stop & Restart")
time.sleep(3)


def press_callback(key):  # callback function for pressing state of certain keys
    global val_w
    global val_a
    global val_s
    global val_d
    global direction
    num_A = 0
    num_D = 0
    if key == KeyCode.from_char('w'):
        print('w is pressed'.format(key.char))

        val_w += 1
    elif key == KeyCode.from_char('a'):
        ser.write(bytes('H', 'UTF-8'))
        print('a is pressed'.format(key.char))
        val_a += 1

    elif key == KeyCode.from_char('s'):
        print('s is pressed: restart'.format(key.char))
        os.system('python snake_control.py') # reload the file
        val_s += 1

    elif key == KeyCode.from_char('d'):
        ser.write(bytes('L', 'UTF-8'))
        print('d is pressed'.format(key.char))
        val_d += 1

    elif key == KeyCode.from_char('r'):
        print('r is pressed: automatic moving')

        while 1:
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('L', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)
            ser.write(bytes('H', 'UTF-8'))
            time.sleep(5)

        





def release_callback(key):  # callback function for the releasing state of keys
    if key == keyboard.Key.esc:
        # Stop listener
        return False


def read_input2(): # this is the master function used for keyboard control
    # x = threading.Thread(target=thread_function)
    # x.start()
    l = keyboard.Listener(on_press=press_callback, on_release=release_callback)
    l.start()
    l.join()


# def thread_function():
#     global line_1, last_received1
#     buffer1 = ''
#     while 1:
#         buffer1 += ser1.read(ser1.inWaiting()).decode()
#         if '\n' in buffer1:
#             last_received1, buffer1 = buffer1.split('\n')[-2:]
#         line_1 = last_received1



if __name__ == '__main__':
    read_input2()  # start keyboard control
