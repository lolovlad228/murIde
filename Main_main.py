from time import time, sleep
from math import fabs
import cv2
import numpy as np
import pymurapi as mur
from threading import Lock, Thread


class Solide(type):

    _instance = None

    _lock = Lock()

    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__call__(*args, **kwargs)
        return cls._instance


def clamp_to180(angle):
    if angle > 180:
        return angle - 360
    if angle < -180:
        return angle + 360
    return angle


def clamp(val, max_val, min_val):
    if val > max_val:
        return max_val
    if val < min_val:
        return min_val
    return val


class PD:
    def __init__(self, k, d):
        self.__kp = k
        self.__dp = d
        self.__prev_error = 0
        self.__timestamp = 0

    def set_p(self, p):
        self.__kp = p

    def set_d(self, d):
        self.__dp = d

    def val_speed(self, error):
        time_now = int(round(time() * 1000))
        k_error = self.__kp * error
        try:
            d_error = self.__dp / (time_now - self.__timestamp) * (error - self.__prev_error)
        except ZeroDivisionError:
            d_error = 0
        self.__prev_error = error
        self.__timestamp = time_now
        return k_error + d_error


def length(x, y):
    try:
        length_now = round(((x ** 1 + y ** 2) ** 0.5), 5)
        return length_now
    except TypeError:
        return 11


class Run:
    def __init__(self, i, count):
        self.__i = i
        self.__count = count
        self.__j = 0

    def add_item(self, i):
        if fabs(self.__i - i) < 0.5:
            if self.__j + 1 >= self.__count:
                return False
            else:
                self.__j += 1
        else:
            self.__i = i
            self.__j = 1
        return True

    def set_count(self, count):
        self.__count = count

class ColorClassif:

    def square_with_a_hole(self, img):
        pass

    __low = None
    __high = None

    def img_approximation(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        low_color = np.array(self.__low, np.uint8)
        high_color = np.array(self.__high, np.uint8)
        img_filter = cv2.inRange(img_hsv, low_color, high_color)
        img = cv2.bitwise_and(img, img, mask=img_filter)
        img = cv2.medianBlur(img, 5)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_filter = cv2.Canny(img_gray, 10, 250)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        closed = cv2.morphologyEx(img_filter, cv2.MORPH_CLOSE, kernel)
        return closed

    def __init__(self, low, high):
        self.__low = low
        self.__high = high

    def cycle(self, img):
        contours = cv2.findContours(self.img_approximation(img), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        cnt_list = []
        max_area = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            line_len = len(cnt)
            if line_len > 50 and len(approx) >= 8:
                ellipse = cv2.fitEllipse(cnt)
                max_area = line_len if line_len > max_area else max_area
                cnt_list.append((int(ellipse[0][0]), int(ellipse[0][1]), line_len, len(approx), approx))
        try:
            squ = list(filter(lambda x: x[3] >= 8 and x[2] == max_area, cnt_list))[-1]
            return True, squ[0] - 160, squ[1] - 120
        except IndexError:
            return False, 0, 0

    def square(self, img):
        img_new = self.img_approximation(img)
        contours = cv2.findContours(img_new, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        center = 0
        angle = 0
        for cnt in contours:

            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = int(rect[1][0] * rect[1][1])

            edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
            edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

            usedEdge = edge1
            if cv2.norm(edge2) > cv2.norm(edge1):
                usedEdge = edge2
            reference = (1, 0)
            angle = 180.0 / np.pi * np.arccos(
                (reference[0] * usedEdge[0] + reference[1] * usedEdge[1]) / (cv2.norm(reference) * cv2.norm(usedEdge)))
            if area > 900:
                center = (int(rect[0][0]), int(rect[0][1]))
        if center == 0:
            return False, 0, 0, 0
        return True, center[0] - 160, center[1] - 120, round(angle)


class Data(metaclass=Solide):
    __yaw = 0
    __depth = 0
    __mur = mur.mur_init()

    def get_yaw(self):
        return self.__yaw

    def get_depth(self):
        return self.__depth

    def set_yaw(self, i):
        self.__yaw = i

    def set_depth(self, i):
        self.__depth = i

    def get_img_bottom(self):
        return self.__mur.get_image_bottom()


class Motor:
    def __init__(self, *koof):
        self.__avi = mur.mur_init()
        self.__pd_depth = PD(koof[0], koof[1])
        self.__pd_keep = PD(koof[2], koof[3])
        self.__pd_yaw = PD(koof[4], koof[5])
        self.__green = ColorClassif((72, 28, 0), (97, 255, 255))
        self.__yellow = ColorClassif((0, 98, 0), (42, 255, 255))
        self.__timestemp = Run(7, 50)

    def keep_depth(self, depth_to_set):
        error = self.__avi.get_depth() - depth_to_set
        power = clamp(self.__pd_depth.val_speed(error), 100, -100)
        self.__avi.set_motor_power(2, power)
        self.__avi.set_motor_power(3, power)

    def keep_yaw(self, yaw_to_set, speed):
        error = clamp_to180(self.__avi.get_yaw() - yaw_to_set)
        power = self.__pd_yaw.val_speed(error)
        self.__avi.set_motor_power(0, clamp(speed - power, 100, -100))
        self.__avi.set_motor_power(1, clamp(speed + power, 100, -100))

    def keep_x_y(self, x, y):
        power_y = clamp(self.__pd_keep.val_speed(y), 50, -50)
        power_x = clamp(self.__pd_keep.val_speed(x), 50, -50)

        self.__avi.set_motor_power(0, -power_y)
        self.__avi.set_motor_power(1, -power_y)
        self.__avi.set_motor_power(4, -power_x)

    def keep_bottom_line(self):
        flag, x, y, angle = self.__yellow.square(Data().get_img_bottom())
        if flag:
            while self.__timestemp.add_item(length(x, y)):
                flag, x, y, angle = self.__yellow.square(Data().get_img_bottom())
                self.keep_x_y(x, y)
                self.keep_depth(Data().get_depth())
                sleep(0.05)
            return True
        else:
            self.keep_yaw(Data().get_yaw(), 30)
            return False

    def keep_bottom_cycle(self):
        flag, x, y = self.__yellow.cycle(Data().get_img_bottom())
        if flag:
            if length(x, y) < 10:
                self.__timestemp.set_count(50)
                while self.__timestemp.add_item(length(x, y)):
                    flag, x, y = self.__yellow.cycle(Data().get_img_bottom())
                    self.keep_x_y(x, y)
                    sleep(0.05)
                return True
            self.keep_x_y(x, y)
            return False
        else:
            self.keep_yaw(Data().get_yaw(), 40)

    def in_cycle(self, speed):
        tm = int(round(time() * 1000))
        flag, x, y, angle = self.__yellow.square(Data().get_img_bottom())
        if flag:
            Data().set_yaw(90 - angle)
            while tm - int(round(time() * 1000)) > -6500:
                self.keep_yaw(Data().get_yaw(), speed)
                self.keep_depth(Data().get_depth())
                sleep(0.05)
            self.__avi.set_motor_power(0, 0)
            self.__avi.set_motor_power(1, 0)

    def move_to_time(self, time_move, speed):
        tm = int(round(time() * 1000))
        while tm - int(round(time() * 1000)) > time_move * -1000:
            self.keep_yaw(Data().get_yaw(), speed)

    def depth_to_time(self, time_depth, h=None):
        tm = int(round(time() * 1000))
        h = Data().get_depth() if h is None else h
        while tm - int(round(time() * 1000)) > time_depth * -1000:
            self.keep_depth(h)


avi = mur.mur_init()
motor = Motor(30, 5, 0.1, 0.3, 0.2, 0.6)
Data().set_depth(1.5)

motor.depth_to_time(4)

while True:
    if motor.keep_bottom_line():
        break

motor.in_cycle(30)

while True:
    if motor.keep_bottom_line():
        break

motor.in_cycle(20)

while True:
    if motor.keep_bottom_cycle():
        break

avi.drop()

motor.depth_to_time(2, 0)


avi.set_motor_power(0, 0)
avi.set_motor_power(1, 0)
avi.set_motor_power(4, 0)