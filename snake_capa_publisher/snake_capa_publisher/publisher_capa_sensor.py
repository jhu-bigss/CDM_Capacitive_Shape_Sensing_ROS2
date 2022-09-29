# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from snake_capa_msg.msg import Capa
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import serial
import time

ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

class CapaPublisher(Node):

    def __init__(self):
        super().__init__('capa_publisher')
        self.publisher_ = self.create_publisher(Capa, 'capa_sensor', 10)
        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Capa()
        line = ser.readline()   # read a byte
        if line:
            if (self.i <= 5):
                self.i += 1
            else:
                string = line.decode()  # convert the byte string to a unicode string
                # print(string)
                mid_data = string.replace("\r\n", "")
                # # print(type(mid_data))
                mid_data = mid_data.split(', ')
                # # print(mid_data)
                # # time.sleep(0.1)
                capa_0 = float(mid_data[0])
                capa_1 = float(mid_data[1])
                capa_2 = float(mid_data[2])
                # print(capa_2)
            # time.sleep(0.1)
            # # msg.data = 'Hello World: %d' % self.i
                msg.capa0 = capa_0
                msg.capa1 = capa_1
                msg.capa2 = capa_2
            # # time.sleep(0.1)
                self.publisher_.publish(msg)
                # self.get_logger().info('Publishing: "%f"' % msg.capa0)
                self.i += 1
        
        
        


def main(args=None):
    rclpy.init(args=args)
    # line = ser.readline()   # read a byte
    # if line:
    #     string = line.decode()  # convert the byte string to a unicode string
    #     mid_data = string.replace("\r\n", "")
    #     print(mid_data)
    capa_publisher = CapaPublisher()

    rclpy.spin(capa_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    capa_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
