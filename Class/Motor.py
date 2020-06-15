import pymurapi as mur
from Class.Math import PD, clamp, clamp_to180, Run, length
from Class.Classificate import ColorClassif
from time import time, sleep
from Class.Data import Data


class Motor:
    def __init__(self, koof):
        self.__avi = mur.mur_init()
        self.__pd_list = koof
        self.__timestemp = Run(7, 50)

    def set_list_pd(self, key, pd):
        self.__pd_list[key] = pd

    def search_cycle(self, img):
        info = ["red", "green", "blue"]
        for i in info:
            flag, x, y, area = Data().get_filter(i).cycle(img)
            if flag:
                return True, i, area
        return False, "None", 0

    def search_platform(self):

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

    def keep_bottom_line(self):
        while True:
            flag, x, y, angle = Data().get_filter("yellow").square(Data().get_img_bottom())
            if flag:
                self.__timestemp.set_k(2)
                while self.__timestemp.add_item(length(x, y)):
                    flag, x, y, angle = Data().get_filter("yellow").square(Data().get_img_bottom())
                    self.keep_x_y(x, y, 0, 1)
                    self.keep_depth(Data().get_depth())
                    sleep(0.05)
                Data().set_yaw(angle)
                print(Data().get_yaw())
                break
            else:
                self.keep_depth(Data().get_depth())
                self.keep_yaw(Data().get_yaw(), 30)
                sleep(0.05)

    def keep_front_cycle(self):
        self.__timestemp.set_k(2)
        while True:
            flag, filter = self.search_cycle(Data().get_img_front())[:2]
            if flag:
                flag, x, y = Data().get_filter(filter).cycle(Data().get_img_front())
                while self.__timestemp.add_item(length(x, y)):
                    flag, x, y = Data().get_filter(filter).cycle(Data().get_img_front())
                    self.keep_x_y(x, y, 2, 3)
                    sleep(0.05)
                break
            else:
                pass

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

