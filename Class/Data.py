from Class.Interfase.ISolid import Solide
import pymurapi as mur


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
