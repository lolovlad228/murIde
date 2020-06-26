from time import time, sleep
from math import fabs
import cv2
import numpy as np
import pymurapi as mur
from threading import Lock, Thread
from operator import itemgetter


class Solide(type):

    _instance = None

    _lock = Lock()

    def __call__(cls, *args, **kwargs):
        with cls._lock:
            if not cls._instance:
                cls._instance = super().__call__(*args, **kwargs)
        return cls._instance


class Data(metaclass=Solide):
    __yaw = 0
    __depth = 0
    __mur = mur.mur_init()
    __filters = {}

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

    def get_img_front(self):
        return self.__mur.get_image_front()

    def get_filter(self, key):
        return self.__filters[key]

    def set_filters(self, key, val):
        self.__filters[key] = val

    def shoot(self):
        return self.__mur.shoot()


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


class ColorClassif:

    __low = None
    __high = None

    def mask(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_filter = cv2.inRange(img_hsv, self.__low, self.__high)
        return img_filter

    def img_approximation_canny(self, img, mask=None):
        img_filter = self.mask(img) if mask is None else mask
        img = cv2.bitwise_and(img, img, mask=img_filter)
        img = cv2.medianBlur(img, 5)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        img_filter = cv2.Canny(img_gray, 10, 20)
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (10, 10))
        closed = cv2.morphologyEx(img_filter, cv2.MORPH_CLOSE, kernel)
        return closed

    def img_approximation_threshold(self, img):
        thresh = cv2.threshold(img, 140, 255, cv2.THRESH_BINARY)[1]
        img = cv2.medianBlur(thresh, 3)
        mask = cv2.inRange(img, self.__low, self.__high)
        return mask

    def __init__(self, low, high):
        self.__low = np.array(low, np.uint8)
        self.__high = np.array(high, np.uint8)


class ImgClasses(metaclass=Solide):
    def quadrangle(self, img):
        contours = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        list_quar = []
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.05 * cv2.arcLength(cnt, True), True)
            rect = cv2.minAreaRect(approx)
            box = cv2.boxPoints(rect)
            area = int(rect[1][0] * rect[1][1])
            if area < 500:
                continue
            list_quar.append((int(rect[0][0]), int(rect[0][1]), int(rect[1][0]) * int(rect[1][1])))
        if len(list_quar) == 0:
            return [[0, 0, 0, 0]]
        else:
            list_quar.sort(key=itemgetter(2), reverse=True)
            return list_quar[:3]

    def left(self, img):
        return img[0:240, 0:160]

    def right(self, img):
        return img[0:240, 160:320]

    def search_platform(self, img):
        platform = {"left": [], "right": []}
        list_filter = ['red', 'green', 'blue', 'yellow', 'white', 'black']
        for i in range(2):
            if i == 0:
                for j in list_filter:
                    qua = self.quadrangle(Data().get_filter(j).img_approximation_canny(self.left(img)))
                    if qua[0][0] == 0:
                        continue
                    platform["left"].append((j, qua[0]))
                platform["left"].sort(key=lambda x: x[1][0])
            elif i == 1:
                for j in list_filter:
                    qua = self.quadrangle(Data().get_filter(j).img_approximation_canny(self.right(img)))
                    if qua[0][0] == 0:
                        continue
                    platform["right"].append((j, qua[0]))
                platform["right"].sort(key=lambda x: x[1][0])
        if len(platform["left"]) != 0:
            print(platform)
            if platform["left"][0][0] == "white" and platform["left"][1][0] == "red" and platform["left"][2][0] == "blue":
                return "left", platform["right"]
            else:
                return "right", platform["left"]
        else:
            return 0

    def stick(self, filter, side):
        mask_old = Data().get_filter(filter[0][0]).mask(Data().get_img_front())
        for i in filter:
            mask = Data().get_filter(i[0]).mask(Data().get_img_front())
            mask_old = cv2.bitwise_or(mask_old, mask)
        canny = Data().get_filter(filter[0][0]).img_approximation_canny(Data().get_img_front(), mask=mask_old)
        x_y = self.quadrangle(canny)
        if side == "right":
            for i in x_y:
                if i[0] <= 170:
                    return i[0] - 160, i[1] - 120, i[2] / 1000
        else:
            for i in x_y:
                if i[0] >= 140:
                    return i[0] - 160, i[1] - 120, i[2] / 1000

    def map_atlas(self, filter):
        mask_old = Data().get_filter(filter[0][0]).mask(Data().get_img_bottom())
        for i in filter:
            mask = Data().get_filter(i[0]).mask(Data().get_img_bottom())
            mask_old = cv2.bitwise_or(mask_old, mask)
        canny = Data().get_filter(filter[0][0]).img_approximation_canny(Data().get_img_bottom(), mask=mask_old)
        return canny

    def square(self, filter):
        contours = cv2.findContours(self.map_atlas(filter), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        center = 0
        angle = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = int(rect[1][0] * rect[1][1])
            if area < 1000:
                continue
            edge1 = np.int0((box[1][0] - box[0][0], box[1][1] - box[0][1]))
            edge2 = np.int0((box[2][0] - box[1][0], box[2][1] - box[1][1]))

            usedEdge = edge1
            if cv2.norm(edge2) > cv2.norm(edge1):
                usedEdge = edge2
            reference = (1, 0)
            angle = 180.0 / np.pi * np.arccos(
                (reference[0] * usedEdge[0] + reference[1] * usedEdge[1]) / (cv2.norm(reference) * cv2.norm(usedEdge)))

            center = (int(rect[0][0]), int(rect[0][1]))
        if center == 0:
            return False, 0, 0, 0
        return True, center[0] - 160, center[1] - 120, round(angle)


class Motor:
    def __init__(self, koof):
        self.__avi = mur.mur_init()
        self.__pd_list = koof
        self.__timestemp = Run(7, 50)

    def set_list_pd(self, key, pd):
        self.__pd_list[key] = pd

    def keep_depth(self, depth_to_set):
        error = self.__avi.get_depth() - depth_to_set
        power = clamp(self.__pd_list["depth"].val_speed(error), 100, -100)
        self.__avi.set_motor_power(2, power)
        self.__avi.set_motor_power(3, power)

    def keep_yaw(self, yaw_to_set, speed):
        error = clamp_to180(self.__avi.get_yaw() - yaw_to_set)
        power = self.__pd_list["yaw"].val_speed(error)
        self.__avi.set_motor_power(0, clamp(speed - power, 100, -100))
        self.__avi.set_motor_power(1, clamp(speed + power, 100, -100))

    def keep_x_y(self, x, y, *id_motor):
        power_y = clamp(self.__pd_list["x_y"].val_speed(y), 50, -50)
        power_x = clamp(self.__pd_list["x_y"].val_speed(x), 50, -50)

        self.__avi.set_motor_power(id_motor[0], -power_y)
        self.__avi.set_motor_power(id_motor[1], -power_y)
        self.__avi.set_motor_power(4, -power_x)

    def move_to_time(self, time_move, speed):
        tm = int(round(time() * 1000))
        while tm - int(round(time() * 1000)) > time_move * -1000:
            self.keep_yaw(Data().get_yaw(), speed)
            sleep(0.05)

    def depth_to_time(self, time_depth, h=None):
        tm = int(round(time() * 1000))
        h = Data().get_depth() if h is None else h
        while tm - int(round(time() * 1000)) > time_depth * -1000:
            self.keep_depth(h)
            sleep(0.05)

    def yaw_to_time(self, time_yaw, yaw=None):
        tm = int(round(time() * 1000))
        yaw = Data().get_yaw() if yaw is None else yaw
        while tm - int(round(time() * 1000)) > time_yaw * -1000:
            self.keep_yaw(yaw, 0)
            sleep(0.05)

    def keep_front_platform(self, di_filter, side, k=0.8, count=50):
        self.__timestemp.set_k(k)
        self.__timestemp.set_count(count)
        while True:
            x_y = ImgClasses().stick(di_filter, side)
            flag = True
            if flag:
                while self.__timestemp.add_item(length(x_y[0], x_y[1])):
                    x_y = ImgClasses().stick(di_filter, side)
                    self.keep_x_y(x_y[0] * 2, x_y[1] * 2, 2, 3)
                    sleep(0.05)
                    #self.move_to_time(1, 40)
                Data().set_depth(self.__avi.get_depth() - 0.07)
                self.__avi.set_motor_power(2, 0)
                self.__avi.set_motor_power(3, 0)
                self.__avi.set_motor_power(4, 0)
                break
            else:
                pass

    def keep_front_quadrangle(self, filter, k=0.7):
        self.__timestemp.set_k(k)
        self.__timestemp.set_count(30)
        while True:
            canny = Data().get_filter(filter).img_approximation_canny(Data().get_img_front())
            x_y = ImgClasses().quadrangle(canny)[0]
            flag = True
            if flag:
                while self.__timestemp.add_item(length(x_y[0] - 160, x_y[1] - 120)):
                    canny = Data().get_filter(filter).img_approximation_canny(Data().get_img_front())
                    x_y = ImgClasses().quadrangle(canny)[0]
                    self.keep_x_y(x_y[0] - 160, x_y[1] - 120, 2, 3)
                    sleep(0.05)
                    # self.move_to_time(1, 40)
                self.__avi.set_motor_power(2, 0)
                self.__avi.set_motor_power(3, 0)
                Data().set_depth(self.__avi.get_depth())
                break
            else:
                pass

    def keep_front_platform_size(self, di_filter, side, wh=60, k=1):
        self.__timestemp.set_k(k)
        self.__timestemp.set_count(60)
        while True:
            x, y, w_h = ImgClasses().stick(di_filter, side)
            flag = True
            if flag:
                while self.__timestemp.add_item(wh - w_h):
                    x, y, w_h = ImgClasses().stick(di_filter, side)
                    error = clamp(wh - w_h, 70, -70)
                    power = self.__pd_list["yaw"].val_speed(error)
                    self.__avi.set_motor_power(0, power)
                    self.__avi.set_motor_power(1, power)
                    self.keep_depth(Data().get_depth())
                    sleep(0.05)
                    # self.move_to_time(1, 40)
                self.__avi.set_motor_power(0, 0)
                self.__avi.set_motor_power(1, 0)

                break
            else:
                pass

    def keep_bottom_line(self, di_filter, side, isangle=True):
        while True:
            flag, x, y, angle = ImgClasses().square(di_filter)
            if flag:
                self.__timestemp.set_k(1)
                self.__timestemp.set_count(50)
                while self.__timestemp.add_item(length(x, y)):
                    flag, x, y = ImgClasses().square(di_filter)[:3]
                    self.keep_x_y(x, y, 0, 1)
                    self.keep_depth(Data().get_depth())
                    print(x, y, angle)
                    sleep(0.05)
                if isangle:
                    if side == "right":
                        Data().set_yaw(90)
                    if side == "left":
                        Data().set_yaw(-90)
                break
            else:
                self.keep_depth(Data().get_depth())
                self.keep_yaw(Data().get_yaw(), 30)
                sleep(0.05)


motor = Motor({"depth": PD(30, 5), "yaw": PD(0.5, 0.8), "x_y": PD(0.1, 1.5)})

red = ColorClassif((0, 139, 0), (16, 255, 255))
white = ColorClassif((0, 0, 40), (255, 7, 255))
blue = ColorClassif((130, 98, 56), (167, 255, 255))

black = ColorClassif((0, 0, 0), (232, 255, 40))
yellow = ColorClassif((23, 193, 65), (47, 250, 255))
green = ColorClassif((63, 165, 95), (104, 250, 255))

Data().set_filters("red", ColorClassif((0, 139, 0), (16, 255, 255)))
Data().set_filters("green", ColorClassif((63, 165, 95), (104, 250, 255)))
Data().set_filters("blue", ColorClassif((130, 98, 56), (167, 255, 255)))
Data().set_filters("yellow", ColorClassif((23, 193, 65), (47, 250, 255)))
Data().set_filters("white", ColorClassif((0, 0, 40), (255, 7, 255)))
Data().set_filters("black", ColorClassif((0, 0, 0), (232, 255, 40)))

di_filter = 0
side = 0
motor.depth_to_time(14, 2.5)

while True:
    if ImgClasses().search_platform(Data().get_img_front()) == 0:
        continue
    else:
        side, di_filter = ImgClasses().search_platform(Data().get_img_front())
        break

print(di_filter, "info", side)
ImgClasses().stick(di_filter, side)
print(1)
motor.keep_front_platform(di_filter, side)
print(2)
motor.keep_front_platform_size(di_filter, side)
print(3)
motor.keep_front_quadrangle(di_filter[1][0])
print(4)
motor.keep_front_platform_size([di_filter[1]], side, 36, 0.8)
motor.keep_front_quadrangle(di_filter[1][0], 0.8)


#motor.depth_to_time(2, Data().get_depth() - 0.1)
#motor.yaw_to_time(6, 0)
#Data().shoot()
motor.depth_to_time(14, 2.6)
motor.yaw_to_time(5, 14)
Data().shoot()
motor.depth_to_time(3, 2.6)
motor.yaw_to_time(10, -14)
Data().shoot()
sleep(0.2)
motor.yaw_to_time(5, 0)
motor.depth_to_time(5, 1)
Data().set_depth(1)
motor.keep_bottom_line([di_filter[0], di_filter[2]], side)
motor.yaw_to_time(10)
motor.move_to_time(15, 10)
motor.keep_bottom_line([("blue", 0)], side, False)
if side == "right":
    motor.move_to_time(2, -4)
else:
    motor.move_to_time(2, 4)
motor.depth_to_time(10, 0)

'''motor.depth_to_time(10, 2.4)
motor.yaw_to_time(5, 8.6)
Data().shoot()
motor.depth_to_time(10, 2.4)
motor.yaw_to_time(5, -8.6)
Data().shoot()'''

'''while True:
    frame = Data().get_img_front()
    img_cen = blue.img_approximation_canny(frame)
    list_qua = ImgClasses().stick(di_filter, side)
    img_copy = frame.copy()
    cv2.circle(img_copy, (list_qua[0] + 160, list_qua[1] + 120), 3, (255, 255, 255), 3)
    key = cv2.waitKey(1)
    cv2.imshow("wtf", img_copy)
    cv2.imshow("canny", img_cen)
    print(ImgClasses().search_platform(frame))
    if key == 27:
        break'''
