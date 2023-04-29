# import math
import numpy as np
# from scipy.interpolate import CubicSpline

import rclpy
from rclpy.node import Node

from snake_capa_msg.msg import Capa
# from tf2_msgs.msg import TFMessage
import matplotlib.pyplot as plt

import time
import cv2
import pyrealsense2 as rs

# import os

_EPS = np.finfo(float).eps * 4.0
cali_width, cali_height = 170, 105
scale = 2
snake_real_length = 104
# urdf = [0.0410297210859283, 0.00132488253734261, 0.00131520544017122, \
#         0.00130552834300195, 0.00129585124582631, 0.00128617414865916, \
#         0.00127649705148352, 0.00126681995431425, 0.00125714285714286, \
#         0.00124746575997359, 0.00123778866279794, 0.00122811156562867, \
#         0.00121843446845728, 0.00120875737128587, 0.00119908027411451, \
#         0.0011894031769431, 0.0011797260797717, 0.00117004898260031, \
#         0.00116037188542892, 0.00115069478825752, 0.00114101769108614, \
#         0.00113295011421438, 0.00112972108592831, 0.00112972108593046, \
#         0.00112972108592616, 0.00112972108592835, 0.00112972108592829, \
#         0.0025297210859283]

# urdf_real_length = [x * 3000 for x in urdf]

sample_pts = np.arange(28)*3.8

def rotMtx(theta):
    c, s = np.cos(theta), np.sin(theta)
    return np.array(((c, -s), (s, c)))

# def visulization(init_x, init_y, img, theta, scale):
#     indexes = np.where(img > 0)
#     x = -indexes[0] + init_y
#     y = indexes[1] - init_x
#     R = rotMtx(theta)
#     res = R@np.r_[y.reshape(1, -1), x.reshape(1, -1)]
#     plt.xlim((-200, 200))
#     plt.ylim((-200, 200))
#     plt.scatter(res[0]*scale, res[1]*scale, s=1)
#     plt.draw()
#     plt.pause(0.05)
 
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
    # img_thinned = cv2.ximgproc.thinning(img_gray)
    return img_gray

# def quaternion_matrix(transform):
#     # Calculate the corresponding transformation function, 
#     # contains rotation and translation.
#     # INPUT:  tf.transform object
#     # OUTPUT: 4*4 transformation matrix [R, t; 0, 1]

#     q = np.array((transform.rotation.x, 
#         transform.rotation.y, 
#         transform.rotation.z, 
#         transform.rotation.w
#         ), dtype=np.float64)

#     nq = np.dot(q, q)
#     if nq < _EPS:
#         return np.identity(4)

#     q *= math.sqrt(2.0 / nq)
#     q = np.outer(q, q)
#     R = np.array((
#         (1.0-q[1, 1]-q[2, 2],     q[0, 1]-q[2, 3],     q[0, 2]+q[1, 3]),
#         (    q[0, 1]+q[2, 3], 1.0-q[0, 0]-q[2, 2],     q[1, 2]-q[0, 3]),
#         (    q[0, 2]-q[1, 3],     q[1, 2]+q[0, 3], 1.0-q[0, 0]-q[1, 1]),
#         ), dtype=np.float64)
    
#     t = np.array((transform.translation.x, 
#         transform.translation.y, 
#         transform.translation.z
#         ), dtype=np.float64)

#     return np.r_[np.c_[R, t.T], np.array([[0,0,0,1]])]

class CapaListener(Node):
    def __init__(self):
        super().__init__('snake_sub')

        self.capa = [0.0, 0.0, 0.0]
        self.cali_pts_i = []
        self.cali_pts_j = []
        self.theta = 0
        self.M = np.ones((3, 4))
        self.ratio = 1

        # Configure depth and color streams
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        # Start streaming
        self.pipeline.start(config)

        self.subscription = self.create_subscription(
            Capa,
            'capa_sensor',
            self.capa_callback,
            10
        )
        self.subscription

        time.sleep(3)
        self.M_flag = True
        self.O_flag = True

        while(self.M_flag):    
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            img_color = np.asanyarray(color_frame.get_data())
            cv2.namedWindow("manual_calibration")
            cv2.setMouseCallback("manual_calibration", self.on_EVENT_LBUTTONDOWN)
            cv2.imshow("manual_calibration", img_color)
            cv2.waitKey(0)
            if (len(self.cali_pts_i) == 4):
                cv2.destroyAllWindows()
                self.correction_M()
                self.M_flag = False

        while(self.O_flag):    
            h, w = img_color.shape[0], img_color.shape[1]
            img_manual = cv2.warpPerspective(img_color, self.M, (h, w))
            cv2.namedWindow("manual_calibration")
            cv2.setMouseCallback("manual_calibration", self.on_EVENT_LBUTTONDOWN)
            cv2.imshow("manual_calibration", img_manual)
            cv2.waitKey(0)
            if (len(self.cali_pts_i) == 6):
                cv2.destroyAllWindows()
                self.correction_O()
                self.O_flag = False

        img_thinned = colorFilter(img_manual, mode='blue')
        self.correction_S(img_thinned)
                

    def capa_callback(self, msg):
        capa0_val = msg.capa0
        capa1_val = msg.capa1
        capa2_val = msg.capa2

        self.capa[0] = capa0_val
        self.capa[1] = capa1_val
        self.capa[2] = capa2_val

        # self.get_logger().info('Capa0: "%f"' % msg.capa0)
        # self.get_logger().info('Capa1: "%f"' % msg.capa1)
        # self.get_logger().info('Capa2: "%f"' % msg.capa2)

        # Wait for a coherent pair of frames: depth and color
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if color_frame:
            # Convert images to numpy arrays
            img_color = np.asanyarray(color_frame.get_data())
            h, w = img_color.shape[0], img_color.shape[1]
            img_manual = cv2.warpPerspective(img_color, self.M, (h, w))
            # cv2.imshow("manual_calibration", img_manual)
            # cv2.waitKey(0)
            img_thinned = colorFilter(img_manual, mode='blue')
            x_real, y_real = self.get_real_pos(img_thinned)
            plt.clf()

        self.plot_simulation(x_real, y_real)

    def get_init_pos(self, img_thinned):
        indexes = np.where(img_thinned > 0)
        if len(indexes) == 0:
            print("No Line Detected")
        x = indexes[1] - self.cali_pts_i[4]
        y = -indexes[0] + self.cali_pts_j[4]
        # R = rotMtx(-self.theta)
        # res = R@np.r_[x.reshape(1, -1), y.reshape(1, -1)]
        x_init, y_init = x/scale, y/scale
        return x_init, y_init
    
    def get_real_pos(self, img_thinned):
        indexes = np.where(img_thinned > 0)
        if len(indexes) == 0:
            print("No Line Detected")
        x = indexes[1] - self.cali_pts_i[4]
        y = -indexes[0] + self.cali_pts_j[4]
        # R = rotMtx(-self.theta)
        # res = R@np.r_[x.reshape(1, -1), y.reshape(1, -1)]
        x_init, y_init = x/scale, y/scale
        x_real, y_real = x_init * self.ratio, y_init * self.ratio
        return x_real, y_real

    def plot_simulation(self, x_real, y_real):
        plt.cla()
        # indexes = np.where(img_thinned > 0)
        # if len(indexes) == 0:
        #     print("No Line Detected")
        # x = indexes[1] - self.cali_pts_i[4]
        # y = -indexes[0] + self.cali_pts_j[4]
        # # R = rotMtx(-self.theta)
        # # res = R@np.r_[x.reshape(1, -1), y.reshape(1, -1)]

        # ## Map to real scale
        # x_real, y_real = x/scale, y/scale
        str_capa_0 = "capa0: " + str(round(self.capa[0], 5))
        str_capa_1 = "capa1: " + str(round(self.capa[1], 5))
        str_capa_2 = "capa2: " + str(round(self.capa[2], 5))
        plt.xlim((-100, 100))
        plt.ylim((0, 200))
        plt.text(40,180, str_capa_0)
        plt.text(40,170, str_capa_1)
        plt.text(40,160, str_capa_2)
        plt.scatter(x_real, y_real)
        plt.draw()
        plt.pause(0.01)

    def correction_M(self):
        first_i, first_j = 100, 100
        point1 = np.array([[self.cali_pts_i[0], self.cali_pts_j[0]], 
                           [self.cali_pts_i[1], self.cali_pts_j[1]], 
                           [self.cali_pts_i[2], self.cali_pts_j[2]], 
                           [self.cali_pts_i[3], self.cali_pts_j[3]]], dtype="float32")
        point2 = np.array([[first_i                 , first_j], 
                           [first_i+scale*cali_width, first_j], 
                           [first_i                 , first_j+scale*cali_height], 
                           [first_i+scale*cali_width, first_j+scale*cali_height]], dtype="float32")
        # point2 = np.array([[100, 100], [420, 100], [100, 340], [420, 340]], dtype="float32")
        self.M = cv2.getPerspectiveTransform(point1, point2)

    def correction_O(self):
        temp_j = self.cali_pts_j[4] - self.cali_pts_j[5]
        temp_i = self.cali_pts_i[4] - self.cali_pts_i[5]        
        self.theta = np.arctan(temp_i/ temp_j)

    def correction_S(self, img_thinned):
        x_real, y_real = self.get_init_pos(img_thinned)
        tip_idx = np.argmax(y_real)
        snake_img_length = (x_real[tip_idx]**2 + y_real[tip_idx]**2)**0.5
        print("tip x is: ", x_real[tip_idx], "\n tip y is: ", y_real[tip_idx])
        self.ratio = snake_real_length/snake_img_length
        print("ratio correction comple! \nratio is: ", self.ratio)

    def on_EVENT_LBUTTONDOWN(self, event, i, j, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            # xy = "%d,%d" % (i, j)
            if np.sum(self.M) == 12:
                if len(self.cali_pts_i) < 4:
                    self.cali_pts_i.append(i)
                    self.cali_pts_j.append(j)
            else:
                if len(self.cali_pts_i) < 6:
                    self.cali_pts_i.append(i)
                    self.cali_pts_j.append(j)
            print(i, j)



def main():
    rclpy.init()
    node = CapaListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()

# class FrameListener(Node):

#     def __init__(self):
#         super().__init__('snake_tf2_frame_listener')
#         self.tf_subscriber = self.create_subscription(TFMessage, 'tf', self.tf_callback, 10)
#         self.pose = []
#         self.a = []
#         self.b = []
#         self.M = np.eye(3)

#         # Configure depth and color streams
#         self.pipeline = rs.pipeline()
#         config = rs.config()

#         config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
#         # Start streaming
#         self.pipeline.start(config)

#         ##################
#         # Initialization #
#         ##################
#         time.sleep(3)
#         self.manual_flag = True
#         self.cali_flag = True

#         while(self.manual_flag):    
#             frames = self.pipeline.wait_for_frames()
#             color_frame = frames.get_color_frame()
#             if not color_frame:
#                 continue
#             img_color = np.asanyarray(color_frame.get_data())
#             cv2.namedWindow("manual_calibration")
#             cv2.setMouseCallback("manual_calibration", self.on_EVENT_LBUTTONDOWN)
#             cv2.imshow("manual_calibration", img_color)
#             cv2.waitKey(0)
#             if (len(self.a) == 6):
#                 cv2.destroyAllWindows()
#                 self.correction(img_color)
#                 self.manual_flag = False
        
#         while(self.cali_flag == True):
#             h, w = img_color.shape[0], img_color.shape[1]
#             img_manual = cv2.warpPerspective(img_color, self.M, (h, w))
#             img_green = colorFilter(img_manual, "green")
#             img_blue = colorFilter(img_manual, "blue")

#             ret, binary = cv2.threshold(img_green, 0, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)
#             num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)

#             [self.init_x, self.init_y] = centroids[1]
#             indexes = np.where(img_blue > 0)
#             x = -indexes[0] + self.init_y
#             y = indexes[1] - self.init_x
#             idx = np.argmax(x)
#             self.theta = np.arctan(y[idx]/ x[idx])
#             R = rotMtx(self.theta)
#             res = R@np.r_[y.reshape(1, -1), x.reshape(1, -1)]
#             self.scale = 141.31/max(res[1])
#             self.cali_flag = False
#         plt.ion()

#     def tf_callback(self, msg):
#         self.pos = []
#         T_prev = np.eye(4)
#         for tf in msg.transforms:
#             T = quaternion_matrix(tf.transform)
#             T_prev = (T_prev@T).reshape(4,4)
#             self.pos.append([T_prev[0, 3], T_prev[1, 3], T_prev[2, 3]])
#         self.pos = np.array(self.pos)
#         self.pos -= self.pos[0]
#         # print(self.pos)

#         # Wait for a coherent pair of frames: depth and color
#         frames = self.pipeline.wait_for_frames()
#         color_frame = frames.get_color_frame()
#         if color_frame:
#             # Convert images to numpy arrays
#             img_color = np.asanyarray(color_frame.get_data())
#             h, w = img_color.shape[0], img_color.shape[1]
#             img_manual = cv2.warpPerspective(img_color, self.M, (h, w))
#             img_thinned = colorFilter(img_manual, mode='blue')
#             plt.clf()

#         self.plot_simulation(img_thinned)

#     def plot_simulation(self, img_thinned):
#         plt.cla()
#         s = 141.31/0.03144

#         indexes = np.where(img_thinned > 0)
#         x = -indexes[0] + self.init_y
#         y = indexes[1] - self.init_x
#         R = rotMtx(self.theta)
#         res = R@np.r_[y.reshape(1, -1), x.reshape(1, -1)]
#         plt.xlim((-200, 200))
#         plt.ylim((-200, 200))

#         cap_x, cap_y = -self.pos[:,0]*s, self.pos[:,2]*s
#         cv_x, cv_y = res[0]*self.scale, res[1]*self.scale

#         # plt.scatter(-self.pos[:,0]*s, self.pos[:,2]*s, marker = 'x')
#         # plt.scatter(res[0]*self.scale, res[1]*self.scale, s=1)

#         y_processed, ind = np.unique(cv_y, return_index = True)
#         x_processed  = cv_x[ind]
#         cs = CubicSpline(y_processed, x_processed)
#         # predict_y = cs(cap_x)
#         # print(cap_y - predict_y)
#         # if not os.path.exists('cv_x.txt'):
#         #     np.savetxt('cv_x.txt', cv_x)
#         # if not os.path.exists('cv_y.txt'):
#         #     np.savetxt('cv_y.txt', cv_y)

#         plot_y = np.linspace(min(y_processed), max(y_processed), num = 100)
#         plot_x = cs(plot_y)

#         plt.scatter(cap_x, cap_y, marker = 'x')
#         plt.scatter(cv_x, cv_y, s=1)

#         #plt.scatter(plot_x, plot_y, s=1)

#         plt.draw()
#         plt.pause(0.01)

#     def correction(self, img):
#         point1 = np.array([[self.a[0], self.b[0]], [self.a[1], self.b[1]], [self.a[2], self.b[2]], [self.a[3], self.b[3]]], dtype="float32")
#         point2 = np.array([[100, 100], [420, 100], [100, 340], [420, 340]], dtype="float32")
#         # point2 = np.array([[100, 100], [420, 100], [100, 340], [420, 340]], dtype="float32")
#         self.M = cv2.getPerspectiveTransform(point1, point2)

#     def on_EVENT_LBUTTONDOWN(self, event, x, y, flags, param):
#         if event == cv2.EVENT_LBUTTONDOWN:
#             xy = "%d,%d" % (x, y)
#             self.a.append(x)
#             self.b.append(y)
#             print(x, y)