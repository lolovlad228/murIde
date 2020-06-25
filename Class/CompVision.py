import cv2
import pymurapi as mur
import numpy as np
from Class.Classificate import ColorClassif
from time import time

mr = mur.mur_init()


def nothing(x):
    pass


def test(img):
    cv2.namedWindow("Tracking")
    cv2.createTrackbar("LowH", "Tracking", 0, 255, nothing)
    cv2.createTrackbar("LowS", "Tracking", 0, 255, nothing)
    cv2.createTrackbar("LowV", "Tracking", 0, 255, nothing)
    cv2.createTrackbar("UnH", "Tracking", 255, 255, nothing)
    cv2.createTrackbar("UnS", "Tracking", 255, 255, nothing)
    cv2.createTrackbar("UnV", "Tracking", 255, 255, nothing)
    while True:
        frame = mr.get_image_front()
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        l_h = cv2.getTrackbarPos("LowH", "Tracking")
        l_s = cv2.getTrackbarPos("LowS", "Tracking")
        l_v = cv2.getTrackbarPos("LowV", "Tracking")

        h_h = cv2.getTrackbarPos("UnH", "Tracking")
        h_s = cv2.getTrackbarPos("UnS", "Tracking")
        h_v = cv2.getTrackbarPos("UnV", "Tracking")

        l_d = np.array([l_h, l_s, l_v])
        h_d = np.array([h_h, h_s, h_v])
        mask = cv2.inRange(hsv, l_d, h_d)
        mask = cv2.medianBlur(mask, 5)
        res = cv2.bitwise_and(frame, frame, mask=mask)

        cv2.imshow("Frame", frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("REs", res)

        key = cv2.waitKey(1)
        if key == 27:
            break

def test_info(img):
    cv2.namedWindow("Tracking")
    cv2.createTrackbar("LowH", "Tracking", 0, 255, nothing)
    cv2.createTrackbar("LowS", "Tracking", 0, 255, nothing)
    cv2.createTrackbar("LowV", "Tracking", 0, 255, nothing)
    cv2.createTrackbar("UnH", "Tracking", 255, 255, nothing)
    cv2.createTrackbar("UnS", "Tracking", 255, 255, nothing)
    cv2.createTrackbar("UnV", "Tracking", 255, 255, nothing)
    while True:
        l_h = cv2.getTrackbarPos("LowH", "Tracking")
        l_s = cv2.getTrackbarPos("LowS", "Tracking")
        l_v = cv2.getTrackbarPos("LowV", "Tracking")

        h_h = cv2.getTrackbarPos("UnH", "Tracking")
        h_s = cv2.getTrackbarPos("UnS", "Tracking")
        h_v = cv2.getTrackbarPos("UnV", "Tracking")
        frame = mr.get_image_front()
        cv2.imwrite("img/front-{0}.jpg".format(0), frame)
        _, thresh = cv2.threshold(frame, 140, 255, cv2.THRESH_BINARY)
        l_d = np.array([l_h, l_s, l_v])
        h_d = np.array([h_h, h_s, h_v])
        mask = cv2.inRange(thresh, l_d, h_d)
        img = cv2.bitwise_and(thresh, thresh, mask=mask)
        img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        _, img_gray = cv2.threshold(img_gray, l_h, 255, cv2.THRESH_BINARY)
        img_new = frame.copy()
        mask_ig = cv2.bitwise_not(mask.copy())
        conturs = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)[0]
        for i in conturs:
            aprox = cv2.approxPolyDP(i, 0.02 * cv2.arcLength(i, True), True)
            print(len(aprox))
            cv2.imshow("t", cv2.drawContours(img_new, [aprox], -1, (0, 0, 0), 4))
        cv2.imshow("Frame", thresh)
        cv2.imshow("Mask", mask)
        cv2.imshow("Mask_frame", img)
        cv2.imshow("Mask_frame+++", mask_ig)
        key = cv2.waitKey(1)
        if key == 27:
            break


red = ColorClassif((0, 139, 0), (16, 255, 255))
white = ColorClassif((0, 0, 40), (255, 7, 255))
blue = ColorClassif((130, 98, 56), (167, 255, 255))

black = ColorClassif((0, 0, 0), (232, 255, 40))
yellow = ColorClassif((23, 193, 65), (47, 250, 255))
green = ColorClassif((63, 165, 95), (104, 250, 255))

def main_classificate():
    while True:
        frame = mr.get_image_front()
        fram_4 = frame[0:240, 0:160]
        fram_5 = frame[0:240, 160:320]
        flag, x, y = blue.square(fram_4, mode="canny")[:3]
        flag, x_1, y_1 = red.square(fram_4, mode="canny")[:3]
        cv2.circle(frame, (x + 160, y + 120), 3, (255, 255, 255), 3)
        cv2.circle(frame, (x_1 + 160, y_1 + 120), 3, (0, 0, 0), 3)
        key = cv2.waitKey(1)
        cv2.imshow("wtf", red.img_approximation_canny(frame))
        cv2.imshow("wtf_1", blue.img_approximation_canny(frame))
        cv2.imshow("front", fram_4)
        cv2.imshow("bot", fram_5)

        if key == 27:
            break


def detect():
    centers = []
    tm = int(round(time() * 1000))
    while tm - int(round(time() * 1000)) > 10 * -1000:
        frame = mr.get_image_front()
        frame = frame[0:240, 0:160]
        flag, x, y = blue.quadrangle(frame)[:3]
        flag, x_1, y_1 = red.quadrangle(frame)[:3]
        cv2.circle(frame, (x, y), 3, (255, 255, 255), 3)
        cv2.circle(frame, (x_1, y_1), 3, (0, 0, 0), 3)
        cv2.imshow("front", frame)
        key = cv2.waitKey(1)
        if key == 27 or x != 0 and x_1 != 0:
            centers.append([x, y])
            centers.append([x_1, y_1])
            break
    if len(centers) < 2:
        return "right"
    if centers[0][0] <= 120 and centers[0][1] < centers[1][1]:
        return "right"
    else:
        return "left"

#print(detect())

test(mr.get_image_bottom())
#main_classificate()

#test_info(mr.get_image_bottom())

#frame = cv2.imread("img/front-0.jpg")
#cv2.imwrite("img/conturs.jpg", frame)
#cv2.imwrite("img/front-1.jpg", red.img_approximation_threshold(frame))
#x, y = red.square(frame, isarea="yas", mode="threshold")[1:3]
#cv2.circle(frame, (x + 160, y + 120), 3, (255, 255, 255), 3)
#cv2.imwrite("img/conturs.jpg", frame)

