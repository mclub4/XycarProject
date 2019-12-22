import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.image_width = 640
        self.scan_width, self.scan_height = 150, 40
        self.lmid, self.rmid = self.scan_width, self.image_width - self.scan_width
        self.area_width, self.area_height = 20, 10
        self.roi_vertical_pos = 270
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        self.pixel_cnt_threshold = 0.01 * self.area_width * self.area_height
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.bridge = CvBridge()
        self.ROI_img = np.zeros(shape=(self.scan_height, self.image_width),
                                dtype=np.uint8)
        self.hough_img = np.zeros(shape=(self.scan_height, self.image_width),
                                  dtype=np.uint8)
        self.hough_img2 = np.zeros(shape=(self.scan_height, self.image_width),
                                   dtype=np.uint8)
        self.see = True
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):

        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 0.6

        lbound = np.array([0, 0, value_threshold + 50], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(self.cam_img, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(self.mask, (5, 5), 0)
        self.edge = cv2.Canny(blur, 70, 190)

    def detect_lines(self):
        self.see = True
        left = -1
        right = 641

        vertices = np.array([[(0, 350), (0, 320), (80, 280), (560, 280), (640, 320), (640, 360), (620, 360), (330, 270),
                              (310, 270), (20, 350)]], dtype=np.int32)

        '''
            edge2 = self.edge

            cv2.line(edge2, (0,350), (0,320), (50,255,100),2)
        cv2.line(edge2, (0,320), (80,280), (50,255,100),2)
        cv2.line(edge2, (80,280), (560,280), (50,255,100),2)
        cv2.line(edge2, (560,280), (640,320), (50,255,100),2)
        cv2.line(edge2, (640,320), (640,360), (50,255,100),2)
        cv2.line(edge2, (640,360), (330,270), (50,255,100),2)
        cv2.line(edge2, (330,270), (310,270), (50,255,100),2)
        cv2.line(edge2, (310,270), (20,350), (50,255,100),2)
        cv2.line(edge2, (20,350), (0,350), (50,255,100),2)
        cv2.imshow('edge2', edge2) 
            '''
        self.ROI_img = self.region_of_interest(self.edge, vertices)
        self.hough_img = self.hough_lines(self.ROI_img, 1, 1 * np.pi / 180, 50, 10, 20)
        self.hough_img2 = cv2.Canny(self.hough_img, 50, 190)
        for l in range(20, 250):
            area = self.hough_img2[285:295, l:l + 20]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                left = l
                break

        for r in range(620, 410, -1):
            area = self.hough_img2[285:295, r - 20:r]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                right = r
                break

        areax = self.mask[362:391, 120:440]
        cv2.imshow("areax", areax)
        if cv2.countNonZero(areax) > (320 * 29 * 0.8):
            left = -2
        # print("left-right", left, right)
        return left, right

    def show_images(self, left, right):
        # Display images for debugging purposes;
        # do not forget to call cv2.waitKey().
        if left != 0:
            lsquare = cv2.rectangle(self.hough_img2,
                                    (left, 290),
                                    (left + 20, 300),
                                    (80, 80, 80), 3)
        if right != 640:
            rsquare = cv2.rectangle(self.hough_img2,
                                    (right - 20, 290),
                                    (right, 300),
                                    (80, 80, 80), 3)
        # cv2.imshow("origin", self.cam_img)
        # cv2.imshow("mask", self.mask)
        cv2.imshow("edge", self.edge)
        cv2.imshow("hough2", self.hough_img2)
        cv2.waitKey(10)

        pass

    def region_of_interest(self, img, vertices, color3=(255, 255, 255), color1=255):
        mask = np.zeros_like(img)
        if len(img.shape) > 2:
            color = color3
        else:
            color = color1

        cv2.fillPoly(mask, vertices, color)
        ROI_image = cv2.bitwise_and(img, mask)

        return ROI_image

    def draw_lines(self, img, lines, color=[0, 0, 255], thickness=2):
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(img, (x1, y1), (x2, y2), color, thickness)

    def hough_lines(self, img, rho, theta, threshold, min_line_len, max_line_gap):
        lines = cv2.HoughLinesP(img, rho, theta, threshold, np.array([]),
                                minLineLength=min_line_len, maxLineGap=max_line_gap)
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)

        if lines != None:
            self.draw_lines(line_img, lines)
            self.see = False

        return line_img

    def weighted_img(self, img, initial_img, a=1, b=1., c=0.):
        return cv2.addWeighted(initial_img, a, img, b, c)

    """def segment_lines(lines, delta):
        v_lines = []
            for line in lines:
            for x1,y1,x2,y2 in line:
                if abs(x2-x1)<delta:
                           v_lines.append(line)
        return v_lines"""
