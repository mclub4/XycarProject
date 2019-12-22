# !/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from imuread import ImuRead


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.imu = ImuRead('/diagnostics')
        self.b_l = 0
        self.b_r = 0
        self.origin = False

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        r, p = self.imu.get_data()
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r, r, p)
        if self.origin and p > -6.5:
            speed = -7
            angle = 0
        elif ((r > 4 and r < 175.0) or (r < -4 and r > -175.0)) and self.origin == False:
            speed = 28  # 30
            angle = -5
        elif p <= -6.5 and p > -7.5:
            speed = -1
            angle = 0
        elif p <= -7.5 and p > -8.5:
            speed = 24
            angle = 0
        elif p <= -8.5 and p >= -15:
            speed = 26  # 28
            angle = 0
        elif p < -15:
            speed = 34.5
            angle = 0
            self.origin = True
        '''
        elif (r>0 and r<175.0) or (r<0 and r>-175.0):
               speed = 30
           angle = -5
        elif line_l == -2:
           speed = 0
           angle = 0
           print("it stopped")
        '''
        print("angle-speed:", angle, speed)
        print("roll:", r, p)
        # print('R (%.1f) P (%.1f), Y (%.1f)' % (r, p, y))
        self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        if left == -1 and right == 641:
            left = self.b_l
            right = self.b_r
        mid = (left + right) // 2

        if mid < 250:
            angle = -82
        elif mid >= 250 and mid < 260:
            angle = -70
        elif mid >= 260 and mid < 270:
            angle = -40
        # angle - -55
        elif mid >= 270 and mid < 280:
            # angle = -50
            angle = -30
        elif mid >= 280 and mid < 285:
            angle = -10
        elif mid > 390:
            angle = 73
        elif mid >= 380 and mid < 390:
            angle = 50
        elif mid >= 370 and mid < 380:
            angle = 38
        elif mid >= 360 and mid < 370:
            angle = 28
        elif mid >= 355 and mid < 360:
            angle = 1
        else:
            angle = 0

        self.b_l = left
        self.b_r = right

        print(self.b_l, self.b_r)
        print(left, right)

        return angle

    def accelerate(self, angle, left, mid, right, roll, pitch):
        after = time.time()
        print(mid, after - now)
        # if mid < 90: #and (after-now) > 70.5:
        #   speed = 20
        # if mid < 85: #and (after-now) > 70.5:
        #   speed = 12
        # if mid < 80: #and (after-now) > 70.5:
        #   speed = 7
        # if mid < 85: #and (after-now) > 70.5:
        #   speed = 4
        if mid < 95 and (after - now) < 20.5:
            speed = 0
        elif angle == 10 or angle == -10:
            speed = 32
        elif angle < -30 or angle > 30:
            speed = 34
        else:
            speed = 35

        return speed

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    now = time.time()
    rate = rospy.Rate(18)

    car.b_l, car.b_r = car.line_detector.detect_lines()  # 55,65,560,570
    while (car.b_l < 30 and car.b_l > 150) and (car.b_r < 640 and car.b_r > 400):
        car.b_l, car.b_r = car.line_detector.detect_lines()
        car.line_detector.show_images(car.b_l, car.b_r)
        print(car.b_l, car.b_r)

    while not rospy.is_shutdown():
        car.trace()
        rate.sleep()
    rospy.on_shutdown(car.exit)
