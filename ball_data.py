import serial
import csv
import numpy as np
import time


class BallData:
    def __init__(self, arduino_port):
        self.__arduino_port = arduino_port
        self.__connection = serial.Serial(self.__arduino_port, 9600, timeout=.01)
        self.__coordinateX = 0.0
        self.__coordinateY = 0.0
        self.__joystickX = 0
        self.__joystickY = 0
        self.__accelX = 0
        self.__accelY = 0
        self.__accelZ = 0
        self.__buttonZ = 0
        self.__buttonC = 0
        self.__sp_coordinateX = 0.0
        self.__sp_coordinateY = 0.0
        self.__last_data_X = np.zeros((3,), dtype=int)
        self.__last_data_Y = np.zeros((3,), dtype=int)
        self.__median = [0, 0]
        self.__timer_flag = 1
        self.__start = time.time()
        self.__data = list()
        self.__coordinateX_raw = 0
        self.__coordinateY_raw = 0

    def filter_data(self):
        self.__last_data_X = self.__last_data_X[1:]
        self.__last_data_X = np.append(self.__last_data_X, self.__coordinateX)
        medianX = int(np.median(self.__last_data_X))
        self.__last_data_Y = self.__last_data_Y[1:]
        self.__last_data_Y = np.append(self.__last_data_Y, self.__coordinateY)
        medianY = int(np.median(self.__last_data_Y))
        return [medianX, medianY]

    def read_coords(self):
        self.__connection.write('1'.encode())
        data = self.__connection.readline()[:-1]
        dec = data.decode('utf-8')
        if data and dec[0] == "*":
            i = 0
            coma_counter = 0
            coordinateX_str = ''
            coordinateY_str = ''
            while i != len(dec):
                if dec[i] == ',':
                    coma_counter += 1
                if coma_counter == 0 and dec[i] != ',':
                    coordinateX_str += dec[i]
                if coma_counter == 1 and dec[i] != ',':
                    coordinateY_str += dec[i]
                i += 1

            coordinateX_str = coordinateX_str[1:]

            if coordinateX_str != '':
                self.__coordinateX_raw = int(float(coordinateX_str))
                self.__coordinateX = (float(self.__coordinateX_raw) - 222) * 378/533 - 189
                if self.__coordinateX > 189:
                    self.__coordinateX = 189
                if self.__coordinateX < -189:
                    self.__coordinateX = -189

            if coordinateY_str != '':
                self.__coordinateY_raw = int(float(coordinateY_str))
                self.__coordinateY = (float(self.__coordinateY_raw) - 209) * 300 / 576 - 150
                if self.__coordinateY > 150:
                    self.__coordinateY = 150
                if self.__coordinateY < -150:
                    self.__coordinateY = -150

            self.__median = self.filter_data()
            return [self.__median[0], self.__median[1]]
        return None

    def read_trajectory(self, mode):
        if mode == 1:
            self.__sp_coordinateX = 0.08 * np.sin(1 * time.time())
            self.__sp_coordinateY = 0.08 * np.cos(1 * time.time())
        elif mode == 2:
            if 0 <= time.time() % 20 < 5:
                self.__sp_coordinateX = 0.08
                self.__sp_coordinateY = 0.08
            elif 5 <= time.time() % 20 < 10:
                self.__sp_coordinateX = 0.08
                self.__sp_coordinateY = -0.08
            elif 10 <= time.time() % 20 < 15:
                self.__sp_coordinateX = -0.08
                self.__sp_coordinateY = -0.08
            else:
                self.__sp_coordinateX = -0.08
                self.__sp_coordinateY = 0.08
        elif mode == 3:
            if 3 <= time.time() % 30 < 18:
                self.__sp_coordinateX = 0.05
                self.__sp_coordinateY = 0.05
            else:
                self.__sp_coordinateX = -0.05
                self.__sp_coordinateY = -0.05
        elif mode == 4:
            joystick = self.read_nunckuk()
            if joystick is not None and 128 >= joystick[0] >= -128 and 128 >= joystick[1] >= -128:
                self.__sp_coordinateX += joystick[0] * 0.000016
                self.__sp_coordinateY += joystick[1] * 0.000016
                if self.__sp_coordinateX > 0.149:
                    self.__sp_coordinateX = 0.149
                if self.__sp_coordinateX < -0.179:
                    self.__sp_coordinateX = -0.179
                if self.__sp_coordinateY > 0.140:
                    self.__sp_coordinateY = 0.140
                if self.__sp_coordinateY < -0.110:
                    self.__sp_coordinateY = -0.110
        else:
            self.__sp_coordinateX = 0
            self.__sp_coordinateY = 0
        return [self.__sp_coordinateX, self.__sp_coordinateY]

    def read_nunckuk(self):
        self.__connection.write('2'.encode())
        data = self.__connection.readline()[:-1]
        dec = data.decode('utf-8')
        if data and dec[0] == "*":
            i = 0
            coma_counter = 0
            joystickX_str = ''
            joystickY_str = ''
            while i != len(dec):
                if dec[i] == ',':
                    coma_counter += 1
                if coma_counter == 0 and dec[i] != ',':
                    joystickX_str += dec[i]
                if coma_counter == 1 and dec[i] != ',':
                    joystickY_str += dec[i]
                i += 1

            joystickX_str = joystickX_str[1:]

            if joystickX_str != '':
                self.__joystickX = int(float(joystickX_str))
            if joystickY_str != '':
                self.__joystickY = int(float(joystickY_str))

            return [self.__joystickX, self.__joystickY]
        return None

    def write_data(self, time_section_in_seconds):
        if self.__timer_flag == 1:
            self.__start = time.time()
            self.__timer_flag = 0
            self.__data = list()

        time_elapsed = time.time() - self.__start
        self.__data.append([time_elapsed, self.__median[0], self.__median[1]])

        if time_elapsed >= time_section_in_seconds:
            self.__timer_flag = 1
            row_count = len(self.__data)
            fieldnames = ["time", "coordinateX", "coordinateY"]
            with open('odczyt.csv', 'w', newline='') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                for i in range(row_count):
                    info = {
                        "time": self.__data[i][0],
                        "coordinateX": self.__data[i][1],
                        "coordinateY": self.__data[i][2]
                    }

                    csv_writer.writerow(info)

    def close_connection(self):
        self.__connection.close()