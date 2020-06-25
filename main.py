from Class.Motor import Motor
from Class.Math import PD
import pymurapi as mur
from Class.Data import Data
from Class.Classificate import ColorClassif
from time import sleep

avi = mur.mur_init()
motor = Motor({"depth": PD(30, 5), "yaw": PD(0.5, 0.8), "x_y": PD(0.1, 0.6)})

Data().set_filters("red", ColorClassif((0, 139, 0), (16, 255, 255)))
Data().set_filters("green", ColorClassif((62, 229, 93), (98, 255, 255)))
Data().set_filters("blue", ColorClassif((30, 185, 211), (255, 255, 255)))
Data().set_filters("yellow", ColorClassif((19, 193, 95), (33, 255, 255)))
Data().set_filters("white", ColorClassif((0, 0, 0), (255, 4, 255)))
Data().set_filters("black", ColorClassif((0, 0, 0), (123, 255, 36)))

motor.depth_to_time(14, 2.3)
motor.keep_front_cycle()


