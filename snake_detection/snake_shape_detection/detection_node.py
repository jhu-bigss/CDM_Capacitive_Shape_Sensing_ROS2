import math
import numpy as np
from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node

from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt

import time
import cv2
import pyrealsense2 as rs

import os


_EPS = np.finfo(float).eps * 4.0


def rotMtx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array(((c, -s), (s, c)))

def visulization(init_x, init_y, img, theta, scale):
    indexes = np.where(img > 0)
    x = -indexes[0] + init_y
    y = indexes[1] - init_x
    R = rotMtx(theta)
    res = R@np.r_[y.reshape(1, -1), x.reshape(1, -1)]
    plt.xlim((-200, 200))
    plt.ylim((-200, 200))
    plt.scatter(res[0]*scale, res[1]*scale, s=1)
    plt.draw()
    plt.pause(0.05)
 
def colorFilter(img_color, mode = "blue"):
    if mode == 'blue':
        lower = np.array([90, 50, 50])
        upper = np.array([150, 255, 255])
    elif mode == 'green':
        lower = np.array([25, 120, 70])
        upper = np.array([95, 255, 255])
    else:
        print("Please choose green or blue mode")
        return
    hsv = cv2.cvtColor(img_color, cv2.COLOR_BGR2HSV)
    mask_filter = cv2.inRange(hsv, lower, upper)
    img_mask = cv2.bitwise_and(hsv, hsv, mask=mask_filter)
    img_gray = cv2.cvtColor(img_mask, cv2.COLOR_RGB2GRAY)
    img_thinned = cv2.ximgproc.thinning(img_gray)
    return img_thinned


def quaternion_matrix(transform):
    # Calculate the corresponding transformation function, 
    # contains rotation and translation.
    # INPUT:  tf.transform object
    # OUTPUT: 4*4 transformation matrix [R, t; 0, 1]

    q = np.array((transform.rotation.x, 
        transform.rotation.y, 
        transform.rotation.z, 
        transform.rotation.w
        ), dtype=np.float64)

    nq = np.dot(q, q)
    if nq < _EPS:
        return np.identity(4)

    q *= math.sqrt(2.0 / nq)
    q = np.outer(q, q)
    R = np.array((
        (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3]),
        (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3]),
        (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1]),
        ), dtype=np.float64)
    
    t = np.array((transform.translation.x, 
        transform.translation.y, 
        transform.translation.z
        ), dtype=np.float64)

    return np.r_[np.c_[R, t.T], np.array([[0,0,0,1]])]

class FrameListener(Node):

    def __init__(self):
        super().__init__('snake_tf2_frame_listener')
        self.tf_subscriber = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
        self.pose = []
        self.a = []
        self.b = []
        self.M = np.eye(3)

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Start streaming
        self.pipeline.start(config)

        ##################
        # Initialization #
        ##################
        time.sleep(3)
        self.manual_flag = True
        self.cali_flag = True

        while(self.manual_flag):    
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            img_color = np.asanyarray(color_frame.get_data())
            cv2.namedWindow("manual_calibration")
            cv2.setMouseCallback("manual_calibration", self.on_EVENT_LBUTTONDOWN)
            cv2.imshow("manual_calibration", img_color)
            cv2.waitKey(0)
            if (len(self.a) == 4):
                cv2.destroyAllWindows()
                self.correction(img_color)
                self.manual_flag = False
        
        while(self.cali_flag == True):
            h, w = img_color.shape[0], img_color.shape[1]
            img_manual = cv2.warpPerspective(img_color, self.M, (h, w))
            img_green = colorFilter(img_manual, "green")
            img_blue = colorFilter(img_manual, "blue")

            ret, binary = cv2.threshold(img_green, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)

            [self.init_x, self.init_y] = centroids[1]
            indexes = np.where(img_blue > 0)
            x = -indexes[0] + self.init_y
            y = indexes[1] - self.init_x
            idx = np.argmax(x)
            self.theta = np.arctan(y[idx]/ x[idx])
            R = rotMtx(self.theta)
            res = R@np.r_[y.reshape(1, -1), x.reshape(1, -1)]
            self.scale = 141.31/max(res[1])
            self.cali_flag = False
        plt.ion()

    def tf_callback(self, msg):
        self.pos = []
        T_prev = np.eye(4)
        for tf in msg.transforms:
            T = quaternion_matrix(tf.transform)
            T_prev = (T_prev@T).reshape(4,4)
            self.pos.append([T_prev[0, 3], T_prev[1, 3], T_prev[2, 3]])
        self.pos = np.array(self.pos)
        self.pos -= self.pos[0]
        # print(self.pos)

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            # Convert images to numpy arrays
            img_color = np.asanyarray(color_frame.get_data())
            h, w = img_color.shape[0], img_color.shape[1]
            img_manual = cv2.warpPerspective(img_color, self.M, (h, w))
            img_thinned = colorFilter(img_manual, mode='blue')
            plt.clf()

        self.plot_simulation(img_thinned)

    def plot_simulation(self, img_thinned):
        plt.cla()
        s = 141.31/0.03144

        indexes = np.where(img_thinned > 0)
        x = -indexes[0] + self.init_y
        y = indexes[1] - self.init_x
        R = rotMtx(self.theta)
        res = R@np.r_[y.reshape(1, -1), x.reshape(1, -1)]
        plt.xlim((-200, 200))
        plt.ylim((-200, 200))

        cap_x, cap_y = -self.pos[:,0]*s, self.pos[:,2]*s
        cv_x, cv_y = res[0]*self.scale, res[1]*self.scale

        # plt.scatter(-self.pos[:,0]*s, self.pos[:,2]*s, marker = 'x')
        # plt.scatter(res[0]*self.scale, res[1]*self.scale, s=1)

        y_processed, ind = np.unique(cv_y, return_index = True)
        x_processed  = cv_x[ind]
        cs = CubicSpline(y_processed, x_processed)
        # predict_y = cs(cap_x)
        # print(cap_y - predict_y)
        # if not os.path.exists('cv_x.txt'):
        #     np.savetxt('cv_x.txt', cv_x)
        # if not os.path.exists('cv_y.txt'):
        #     np.savetxt('cv_y.txt', cv_y)

        plot_y = np.linspace(min(y_processed), max(y_processed), num = 100)
        plot_x = cs(plot_y)

        plt.scatter(cap_x, cap_y, marker = 'x')
        plt.scatter(cv_x, cv_y, s=1)

        #plt.scatter(plot_x, plot_y, s=1)

        plt.draw()
        plt.pause(0.01)

    def correction(self, img):
        point1 = np.array([[self.a[0], self.b[0]], [self.a[1], self.b[1]], [self.a[2], self.b[2]], [self.a[3], self.b[3]]], dtype="float32")
        point2 = np.array([[100, 100], [420, 100], [100, 340], [420, 340]], dtype="float32")
        self.M = cv2.getPerspectiveTransform(point1, point2)

    def on_EVENT_LBUTTONDOWN(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            xy = "%d,%d" % (x, y)
            self.a.append(x)
            self.b.append(y)
            print(x, y)

def main():
    rclpy.init()
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()