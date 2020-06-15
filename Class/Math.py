from time import time
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
        length_now = round(((x ** 2 + y ** 2) ** 0.5), 5)
        return length_now
    except TypeError:
        return 11


class Run:
    def __init__(self, i, count):
        self.__i = i
        self.__count = count
        self.__j = 0
        self.__k = 0.5

    def add_item(self, i):
        if fabs(self.__i - i) < self.__k:
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

    def set_k(self, k):
        self.__k = k

