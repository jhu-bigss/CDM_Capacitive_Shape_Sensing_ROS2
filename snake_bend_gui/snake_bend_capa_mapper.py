from ctypes import alignment
from re import S

import math
import threading
import sys
from capa_interfaces.msg import Num

# from python_qt_binding import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

# Global variables
mapper_counter = 0
capa0_temp, capa1_temp, capa2_temp = 0.0, 0.0, 0.0

class MapPubSub(Node):
    def __init__(self):
        super().__init__('capa_mapper')
        self.subscription = self.create_subscription(
            Num,
            'capa_sensor',
            self.mapper_listener_callback,
            10)
        self.subscription
        
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.joint_states = JointState()
        

    mapper_counter = 0  # initialization counter
    capa0_temp, capa1_temp, capa2_temp = 0.0, 0.0, 0.0

    def mapper_listener_callback(self, msg):
        # self.get_logger().info('Data read from capa_interfaces.')
        global mapper_counter
        global capa0_temp, capa1_temp, capa2_temp

        capa0_val = msg.capa0
        capa1_val = msg.capa1
        capa2_val = msg.capa2
        
        if mapper_counter < 50:
            capa0_temp = capa0_temp + capa0_val
            capa1_temp = capa1_temp + capa1_val
            capa2_temp = capa2_temp + capa2_val
            print(f"Calibrating: {mapper_counter}/50 !!")
            mapper_counter += 1

        elif (mapper_counter == 50):  # find the middle values
            capa0_temp = capa0_temp/50.0
            capa1_temp = capa1_temp/50.0
            capa2_temp = capa2_temp/50.0
            mapper_counter += 1
            print("Calibration Done!")
        

        else: # publish to the joint state
            # print((math.pi/180)*0.1*(-43.27*(capa0_val - capa0_temp)))
            # print((math.pi/180)*0.12987*(-48.808*(capa1_val - capa1_temp) - 63.72+63.72))
            # print((math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)-7.6044))
            self.joint_states.header.stamp = self.get_clock().now().to_msg()
            self.joint_states.name = ['joint_00',
                                    'joint_01',
                                    'joint_02',
                                    'joint_03',
                                    'joint_04',
                                    'joint_05',
                                    'joint_06',
                                    'joint_07',
                                    'joint_08',
                                    'joint_09',
                                    'joint_10',
                                    'joint_11',
                                    'joint_12',
                                    'joint_13',
                                    'joint_14',
                                    'joint_15',
                                    'joint_16',
                                    'joint_17',
                                    'joint_18',
                                    'joint_19',
                                    'joint_20',
                                    'joint_21',
                                    'joint_22',
                                    'joint_23',
                                    'joint_24',
                                    'joint_25',
                                    'joint_26']

            self.joint_states.position = [(math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.13333*(33.018*(capa2_val - capa2_temp)),  #bottom node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.12987*(48.808*(capa1_val - capa1_temp)),  #middle node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp)),  #top node
                                      (math.pi/180)*0.1*(43.27*(capa0_val - capa0_temp))]  #top node

            self.publisher.publish(self.joint_states)
        
        


def main(args=None):
    rclpy.init(args=args)
    snake_mapper = MapPubSub()
    rclpy.spin(snake_mapper)

    snake_mapper.destory_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()