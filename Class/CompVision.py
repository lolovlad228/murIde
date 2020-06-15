import cv2
import pymurapi as mur
import numpy as np
from Class.Classificate import ColorClassif

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
            cv2.imshow("t", cv2.drawContours(img_new, [aprox], -1, (255, 255, 255), 4))
        cv2.imshow("Frame", thresh)
        cv2.imshow("Mask", mask)
        cv2.imshow("Mask_frame", img)
        cv2.imshow("Mask_frame+++", mask_ig)
        key = cv2.waitKey(1)
        if key == 27:
            break


yellow = ColorClassif((0, 98, 0), (42, 255, 255))

red = ColorClassif((0, 0, 5), (171, 255, 255))
green = ColorClassif((0, 11, 0), (255, 255, 255))
blue = ColorClassif((35, 0, 0), (255, 204, 255))

filter_list = [green, yellow]

list_center = []


def main_classificate():
    while True:
        frame = mr.get_image_front()
        frame_1 = mr.get_image_bottom()
        flag, x, y = red.cycle(frame)
        flag_1, x1, y1, angle = yellow.square(frame_1)
        cv2.circle(frame, (x + 160, y + 120), 3, (255, 255, 255), 3)
        cv2.circle(frame_1, (x1 + 160, y1 + 120), 3, (255, 255), 3)
        key = cv2.waitKey(1)
        cv2.imshow("wtf", red.img_approximation_threshold(frame))
        cv2.imshow("front", frame)
        cv2.imshow("bot", frame_1)
        if key == 27:
            break


#test(mr.get_image_bottom())
main_classificate()
#test_info(mr.get_image_bottom())