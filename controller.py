import time
import csv
import numpy as np


class PIDRegulator:
    def __init__(self, kp, ki, kd, ts, set_point, control_value_limit, control_value_rate_of_change_limit):
        self.__kp = kp
        self.__ki = ki
        self.__kd = kd
        self.__ts = ts
        self.__last_time = 0
        self.__set_point = set_point
        self.__control_value = [0, 0]
        self.__control_value_limit = control_value_limit
        self.__control_value_last = [0, 0]
        self.__control_value_roc_limit = control_value_rate_of_change_limit
        self.__error = [0, 0]
        self.__last_error = [0, 0]
        self.__sum = [0, 0]
        self.__timer_flag = 1
        self.__start = time.time()
        self.__start_ts = time.time()
        self.__data = list()
        self.__derivative = [0, 0]
        self.__last_derivative_X = np.zeros((4,), dtype=int)
        self.__last_derivative_Y = np.zeros((4,), dtype=int)
        self.__derivative_mean = [0, 0]
        self.__last_process_value = [0, 0]
        self.__velocity = [0, 0]

    def update_set_point(self, set_point):
        self.__set_point = set_point

    def update(self, process_value):
        self.__ts = time.time() - self.__start_ts
        self.__start_ts = time.time()
        self.__error = [self.__set_point[0] - process_value[0], self.__set_point[1] - process_value[1]]
        self.__sum = [self.__sum[0] + self.__error[0] * self.__ts, self.__sum[1] + self.__error[1] * self.__ts]
        self.__derivative = [(self.__error[0] - self.__last_error[0]) / self.__ts, (self.__error[1] - self.__last_error[1]) / self.__ts]

        self.__last_derivative_X = self.__last_derivative_X[1:]
        self.__last_derivative_X = np.append(self.__last_derivative_X, self.__derivative[0])
        self.__derivative_mean[0] = np.average(self.__last_derivative_X)
        self.__last_derivative_Y = self.__last_derivative_Y[1:]
        self.__last_derivative_Y = np.append(self.__last_derivative_Y, self.__derivative[1])
        self.__derivative_mean[1] = np.average(self.__last_derivative_Y)
        self.__control_value = [self.__kp * self.__error[0] + self.__sum[0] * self.__ki + self.__derivative_mean[0] * self.__kd,
                                self.__kp * self.__error[1] + self.__sum[1] * self.__ki + self.__derivative_mean[1] * self.__kd]

        if self.__control_value[0] > self.__control_value_limit:
            self.__control_value[0] = self.__control_value_limit
        elif self.__control_value[0] < -self.__control_value_limit:
            self.__control_value[0] = -self.__control_value_limit
        if self.__control_value[1] > self.__control_value_limit:
            self.__control_value[1] = self.__control_value_limit
        elif self.__control_value[1] < -self.__control_value_limit:
            self.__control_value[1] = -self.__control_value_limit

        if abs(self.__control_value[0] - self.__control_value_last[0]) > self.__control_value_roc_limit * self.__ts:
            if self.__control_value[0] - self.__control_value_last[0] > 0:
                self.__control_value[0] = self.__control_value_last[0] + (self.__control_value_roc_limit * self.__ts)
            elif self.__control_value[0] - self.__control_value_last[0] < 0:
                self.__control_value[0] = self.__control_value_last[0] - (self.__control_value_roc_limit * self.__ts)

        if abs(self.__control_value[1] - self.__control_value_last[1]) > self.__control_value_roc_limit * self.__ts:
            if self.__control_value[1] - self.__control_value_last[1] > 0:
                self.__control_value[1] = self.__control_value_last[1] + (self.__control_value_roc_limit * self.__ts)
            elif self.__control_value[1] - self.__control_value_last[1] < 0:
                self.__control_value[1] = self.__control_value_last[1] - (self.__control_value_roc_limit * self.__ts)

        self.__velocity = [(process_value[0] - self.__last_process_value[0])/self.__ts,
                           (process_value[1] - self.__last_process_value[1])/self.__ts]

        self.__last_error = self.__error
        self.__control_value_last = self.__control_value
        self.__last_process_value = process_value

        return self.__control_value

    def update_PI_D(self, process_value):
        self.__ts = time.time() - self.__start_ts
        self.__start_ts = time.time()
        self.__error = [self.__set_point[0] - process_value[0], self.__set_point[1] - process_value[1]]

        self.__velocity = [(process_value[0] - self.__last_process_value[0]) / self.__ts,
                           (process_value[1] - self.__last_process_value[1]) / self.__ts]

        self.__sum = [self.__sum[0] + self.__error[0] * self.__ts, self.__sum[1] + self.__error[1] * self.__ts]
        self.__derivative = self.__velocity

        self.__last_derivative_X = self.__last_derivative_X[1:]
        self.__last_derivative_X = np.append(self.__last_derivative_X, self.__derivative[0])
        self.__derivative_mean[0] = np.average(self.__last_derivative_X)
        self.__last_derivative_Y = self.__last_derivative_Y[1:]
        self.__last_derivative_Y = np.append(self.__last_derivative_Y, self.__derivative[1])
        self.__derivative_mean[1] = np.average(self.__last_derivative_Y)
        self.__control_value = [
            self.__kp * self.__error[0] + self.__sum[0] * self.__ki - self.__derivative_mean[0] * self.__kd,
            self.__kp * self.__error[1] + self.__sum[1] * self.__ki - self.__derivative_mean[1] * self.__kd]

        if self.__control_value[0] > self.__control_value_limit:
            self.__control_value[0] = self.__control_value_limit
        elif self.__control_value[0] < -self.__control_value_limit:
            self.__control_value[0] = -self.__control_value_limit
        if self.__control_value[1] > self.__control_value_limit:
            self.__control_value[1] = self.__control_value_limit
        elif self.__control_value[1] < -self.__control_value_limit:
            self.__control_value[1] = -self.__control_value_limit

        if abs(self.__control_value[0] - self.__control_value_last[0]) > self.__control_value_roc_limit * self.__ts:
            if self.__control_value[0] - self.__control_value_last[0] > 0:
                self.__control_value[0] = self.__control_value_last[0] + (self.__control_value_roc_limit * self.__ts)
            elif self.__control_value[0] - self.__control_value_last[0] < 0:
                self.__control_value[0] = self.__control_value_last[0] - (self.__control_value_roc_limit * self.__ts)

        if abs(self.__control_value[1] - self.__control_value_last[1]) > self.__control_value_roc_limit * self.__ts:
            if self.__control_value[1] - self.__control_value_last[1] > 0:
                self.__control_value[1] = self.__control_value_last[1] + (self.__control_value_roc_limit * self.__ts)
            elif self.__control_value[1] - self.__control_value_last[1] < 0:
                self.__control_value[1] = self.__control_value_last[1] - (self.__control_value_roc_limit * self.__ts)

        self.__last_error = self.__error
        self.__control_value_last = self.__control_value
        self.__last_process_value = process_value

        return self.__control_value

    def write_data(self, time_section_in_seconds):
        if self.__timer_flag == 1:
            self.__start = time.time()
            self.__timer_flag = 0
            self.__data = list()

        time_elapsed = time.time() - self.__start
        self.__data.append([time_elapsed, self.__error[0] * self.__kp, self.__error[1] * self.__kp,
                            self.__sum[0] * self.__ki, self.__sum[1] * self.__ki, self.__derivative_mean[0] * self.__kd,
                            self.__derivative_mean[1] * self.__kd, self.__control_value[0], self.__control_value[1],
                            self.__velocity[0], self.__velocity[1], self.__set_point[0], self.__set_point[1]])

        if time_elapsed >= time_section_in_seconds:
            self.__timer_flag = 1
            row_count = len(self.__data)
            fieldnames = ["czas", "uchybX", "uchybY", "calkaX", "calkaY", "pochodnaX", "pochodnaY",
                          "sterowanieX", "sterowanieY", "predkoscX", "predkoscY", "zadanaX", "zadanaY"]
            with open('sterowanie.csv', 'w', newline='') as csv_file:
                csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
                csv_writer.writeheader()
                for i in range(row_count):
                    info = {
                        "czas": self.__data[i][0],
                        "uchybX": self.__data[i][1],
                        "uchybY": self.__data[i][2],
                        "calkaX": self.__data[i][3],
                        "calkaY": self.__data[i][4],
                        "pochodnaX": self.__data[i][5],
                        "pochodnaY": self.__data[i][6],
                        "sterowanieX": self.__data[i][7],
                        "sterowanieY": self.__data[i][8],
                        "predkoscX": self.__data[i][9],
                        "predkoscY": self.__data[i][10],
                        "zadanaX": self.__data[i][11],
                        "zadanaY": self.__data[i][12]
                    }

                    csv_writer.writerow(info)