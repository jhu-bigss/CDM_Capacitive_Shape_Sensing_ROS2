import serial
from pynput import keyboard
from pynput.keyboard import KeyCode
import csv
import threading
import time

# global variables
flag = False  # for keyboard input
line_1 = 0  # value read from snake
line_2 = 0  # value read from encoder
last_received1 = ''
last_received2 = ''

# initialize the serial communication
ser1 = serial.Serial("COM13", 9600, timeout=0.3)  # snake COM port
ser2 = serial.Serial("COM3", 9600, timeout=0.3)  # calibration tool COM port


def press_callback(key):  # trigger for starting sampling from 2 COM port
    global flag
    global line_1, line_2

    if key == KeyCode.from_char('c'):
        flag = True
        print("(C) is pressed, start sampling.")
    elif key == KeyCode.from_char('s'):
        flag = False
        print('(S) Sample stop due to user input')

    if flag:
        for i in range(500):
            time.sleep(0.2)  # sampling rate change here
            # print(type(line_2))
            str1 = line_1
            str2 = line_2
            data1 = str1.replace("\r\n", "").split(', ')
            encoder1, encoder2 = str2.replace("\r\n", '').split(', ')
            encoder1 = -float(encoder1)
            encoder2 = -float(encoder2)
            data2 = str(encoder1 + encoder2)
            data1.append(data2)
            with open('C:/Users/transferRNA/Desktop/snake capa/data/joint01_01.csv', 'a', newline='') as file:
                writer = csv.writer(file, delimiter=',')
                writer.writerow(data1)
            temp = i + 1
            print(f"{temp}/500 Done!")


def release_callback(key):  # command for stopping sampling
    if key == keyboard.Key.esc:
        print("Stop sampling program.")
        return False


def read_calibrate():
    x = threading.Thread(target=thread_function)  # include the reading value thread
    x.start()
    # enable the keyboard listener
    kl = keyboard.Listener(on_press=press_callback, on_release=release_callback)
    kl.start()
    kl.join()


def thread_function():  # value reading thread
    global line_1, line_2, last_received1, last_received2
    buffer1 = ''
    buffer2 = ''
    while 1:
        buffer1 += ser1.read(ser1.inWaiting()).decode()
        buffer2 += ser2.read(ser2.inWaiting()).decode()
        if '\n' in buffer1:
            last_received1, buffer1 = buffer1.split('\n')[-2:]
        if '\n' in buffer2:
            last_received2, buffer2 = buffer2.split('\n')[-2:]
        line_1 = last_received1
        line_2 = last_received2


if __name__ == '__main__':
    read_calibrate()
