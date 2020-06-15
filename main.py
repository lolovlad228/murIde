from Class.Motor import Motor
from Class.Math import PD
import pymurapi as mur
from Class.Data import Data
from Class.Classificate import ColorClassif

avi = mur.mur_init()
motor = Motor({"depth": PD(30, 5), "yaw": PD(0.5, 0.8), "x_y": PD(0.1, 0.6)})

Data().set_filters("red", ColorClassif((0, 0, 5), (171, 255, 255)))
Data().set_filters("green", ColorClassif((0, 11, 0), (255, 255, 255)))
Data().set_filters("blue", ColorClassif((35, 0, 0), (255, 204, 255)))
Data().set_filters("yellow", ColorClassif((0, 98, 0), (42, 255, 255)))

Data().set_depth(2.5)

Data().set_yaw(0)

motor.depth_to_time(6)
motor.keep_bottom_line()
motor.yaw_to_time(10)


motor.move_to_time(5, 40)
Data().set_depth(2.6)
motor.keep_bottom_line()
print(motor.search_platform())
#motor.keep_front_cycle()
#motor.depth_to_time(4, 0)

#  while True:
#    if motor.keep_front_cycle():
#        break

