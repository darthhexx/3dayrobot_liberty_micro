import cv2
import numpy as np
import time
from scipy.spatial.distance import euclidean as dist

class SuperRoo_TargetFinder:
    def __init__(self, init_camera=True, display_windows=False):
        # TUNING PARAMS
        self.erode_iterations = 2
        self.dilate_iterations = 5
        self.target_size = 300
        self.heading_scale_factor = (60.0/2) / (1920.0/2)
        if init_camera:
            self.cap = cv2.VideoCapture(0)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
            self.cap.set(cv2.CAP_PROP_FPS, 30)
        # CANNOT GET WHITE BALANCE WORKING IN OPENCVre
        # IN V4L2UCP set whitebalance auto - off, update color temp
        # TODO research if I can do this cmd line and just run init calls during setup

        # white = cap.get(cv2.CAP_PROP_XI_AUTO_WB)
        # cap.set(cv2.CAP_PROP_XI_AUTO_WB,1.0)
        # white2 = cap.get(cv2.CAP_PROP_XI_AUTO_WB)
        # red = cap.get(cv2.CAP_PROP_WHITE_BALANCE_RED_V)

        self.record_images = False
        self.read_camera = False
        self.display_windows = display_windows
        self.angle = 0.0
        return

    def __del__(self):
        self.cap.release()
        if self.display_windows:
            cv2.destroyAllWindows()

    def grab_image(self):
        _, frame = self.cap.read()
        if self.record_images:
            import os
            image_dir = 'images'
            if not os.path.exists(image_dir):
                os.makedirs(image_dir)
            cv2.imwrite(os.path.join(image_dir, str(time.time()) + ".png"), frame)

        return frame

    def find_target(self, frame):
        '''returns yaw, centre zero, positive right. None if no target found'''
        # Convert BGR to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # define range of blue color in HSV
        # HUE IS 0 to 180
        lower_orange = np.array([5, 150, 100])
        upper_orange = np.array([20, 255, 255])
        low_lower_red = np.array([2, 100, 70])
        low_upper_red = np.array([5, 150, 150])
        high_lower_red = np.array([160, 100, 30])
        high_upper_red = np.array([200, 240, 240])

        # Threshold the HSV image to get only blue colors
        lmask = cv2.inRange(hsv, low_lower_red, low_upper_red)
        hmask = cv2.inRange(hsv, high_lower_red, high_upper_red)
        mask = cv2.bitwise_or(lmask,hmask)
        # Bitwise-AND mask and original image
        res = cv2.bitwise_and(frame, frame, mask=mask)

        er_mask = cv2.erode(mask, None, iterations=self.erode_iterations)
        dl_mask = cv2.dilate(er_mask, None, iterations=self.dilate_iterations)

        # load the image, convert it to grayscale, blur it slightly,
        # and threshold it
        # find contours in the thresholded image
        cnts = cv2.findContours(dl_mask.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = cnts[1]
        image = frame.copy()
        # loop over the contours
        target_list = []

        for c in cnts:
            # compute the center of the contour
            M = cv2.moments(c)
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])

            # draw the contour and center of the shape on the image
            cv2.drawContours(image, [c], -1, (0, 255, 0), 2)
            cv2.circle(image, (cX, cY), 7, (255, 255, 255), -1)
            if c.size < self.target_size:
                cv2.putText(image, "small", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
            else:
                cv2.putText(image, "large", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                target_list.append([c.size, cX, cY])

        targets = np.array(target_list)
        if len(targets) == 0:
            return None
        target_idx = np.argmax(targets,axis=0)
        if len(target_idx) > 1:
            target_idx = target_idx[0]
        largest_target = [int(targets[target_idx][1]), int(targets[target_idx][2])]
        #largest_target = targets.max(axis=0)
        target_x = largest_target[1]
        image_height, image_width, _ = [frame.shape[0], frame.shape[1], frame.shape[2]]
        bottom_point = np.array([round(image_width/2), round(image_height)])

        cv2.line(image, (largest_target[0], largest_target[1]), (bottom_point[0], bottom_point[1]), (255,255,255), 17, -1)
        if self.display_windows:
            cv2.imshow('im_with_keypoints', cv2.resize(image,dsize=(0,0),fx=0.5,fy=0.5))
            cv2.imshow('mask', cv2.resize(mask, dsize=(0, 0), fx=0.5, fy=0.5))
        pixel_x_location = target_x - image_width / 2
        yaw_offset = pixel_x_location * self.heading_scale_factor
        z = round(dist(largest_target, bottom_point))
        x = image_height - largest_target[1]

        self.angle = np.arcsin(x/z)
        return

    def main_loop(self):
        while True:
            frame = self.grab_image()
            self.angle = self.find_target(frame)
        return

    def poll(self):
        return self.angle

