import cv2
import numpy as np
from Class.Interfase.IClassesification import IClassesificationType
from math import fabs


class ColorClassif:

    __low = None
    __high = None

    def img_approximation_canny(self, img):
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        img_filter = cv2.inRange(img_hsv, self.__low, self.__high)
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

    def cycle(self, img, mode="threshold"):
        if mode == "threshold":
            contours = cv2.findContours(self.img_approximation_threshold(img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        else:
            contours = cv2.findContours(self.img_approximation_canny(img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        cnt_list = []
        max_area = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            line_len = len(cnt)
            if line_len > 20 and len(approx) > 4:
                ellipse = cv2.fitEllipse(cnt)
                max_area = line_len if line_len > max_area else max_area
                cnt_list.append((int(ellipse[0][0]), int(ellipse[0][1]), line_len, len(approx), approx))
        try:
            squ = list(filter(lambda x: x[3] > 4 and x[2] == max_area, cnt_list))[-1]
            return True, squ[0] - 160, squ[1] - 120, max_area
        except IndexError:
            return False, 0, 0, 0

    def square(self, img, mode="canny", isarea="no"):
        if mode == "threshold":
            contours = cv2.findContours(self.img_approximation_threshold(img), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[0]
        else:
            contours = cv2.findContours(self.img_approximation_canny(img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        center = 0
        angle = 0
        area = 0
        w_h = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            rect = cv2.minAreaRect(cnt)
            box = cv2.boxPoints(rect)
            box = np.int0(box)
            area = int(rect[1][0] * rect[1][1])
            w_h = (rect[1][0], rect[1][1])
            if area < 2000:
                continue
            if len(approx) is not 4:
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
        if isarea == "no":
            if center == 0:
                return False, 0, 0, 0
            return True, center[0], center[1], round(angle)
        else:
            if center == 0:
                return False, 0, 0, 0, 0, 0
            return True, center[0], center[1], round(angle), area, w_h

    def triangle(self, img, mode="canny", isarea="no"):
        if mode == "threshold":
            contours = cv2.findContours(self.img_approximation_threshold(img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        else:
            contours = cv2.findContours(self.img_approximation_canny(img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        center = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
            print(len(approx))
            frame = cv2.imread("img/conturs.jpg")
            frame = cv2.drawContours(frame, [approx], -1, (0, 0, 0), 4)
            cv2.imwrite("img/conturs.jpg", frame)
            if len(approx) == 6:
                rect = cv2.minAreaRect(cnt)
                center = (int(rect[0][0]), int(rect[0][1]))
            else:
                continue
        if center == 0:
            return False, 0, 0
        else:
            return True, center[0], center[1]

    def quadrangle(self, img):
        contours = cv2.findContours(self.img_approximation_canny(img), cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)[0]
        center = 0
        w_h = 0
        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.01 * cv2.arcLength(cnt, True), True)
            rect = cv2.minAreaRect(approx)
            box = cv2.boxPoints(rect)
            area = int(rect[1][0] * rect[1][1])
            if area < 500:
                continue
            w_h = (int(rect[1][0]), int(rect[1][1]))
            center = (int(rect[0][0]), int(rect[0][1]))
        if center == 0:
            return False, 0, 0, 0
        else:
            return True, center[0], center[1], w_h

    def left(self, img):
        return img[0:240, 0:160]

    def right(self, img):
        return img[0:240, 160:320]


