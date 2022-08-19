from ctypes import alignment

import threading
import sys

from python_qt_binding import QtCore, QtWidgets

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

slider_val = 0
ros_position = 0

class snakeWidgets(QtWidgets.QWidget):
    def __init__(self):
        global slider_val
        super().__init__()
        self.button = QtWidgets.QPushButton("Recenter Snake")
        self.text = QtWidgets.QLabel("Snake Shape Commanding Tool",
                                     alignment=QtCore.Qt.AlignCenter)
        self.text2 = QtWidgets.QLabel("Move the slider to change the shape of the snake",
                                        alignment=QtCore.Qt.AlignCenter)
        
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(-50,50)
        self.slider.setValue(0)
        self.slider.setFixedWidth(200)
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.addWidget(self.text)
        self.layout.addWidget(self.text2)
        self.layout.addWidget(self.button)
        self.layout.addWidget(self.slider, alignment=QtCore.Qt.AlignCenter)

        self.button.clicked.connect(self.reset_slider) # Set the slider value to center


    @QtCore.Slot()
    def reset_slider(self):
        self.slider.setValue(0)
        

class MyNode(QtCore.QObject):
    def __init__(self):
        super().__init__()
        self.node = rclpy.create_node('cable_gui')

        self.publisher_joint_states = self.node.create_publisher(JointState, "/joint_states", 10)

        self.joint_states = JointState()
        self.publish_joint_states(0)

        self.shutdown_requested = False

    @QtCore.pyqtSlot(int)
    def publish_joint_states(self, input_value):
        self.joint_states.header.stamp = self.node.get_clock().now().to_msg()
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
        ros_position = 0.088 * input_value/50.0
        self.joint_states.position = [0.948*ros_position,
                                      0.95*ros_position,
                                      0.952*ros_position,
                                      0.954*ros_position,
                                      0.956*ros_position,
                                      0.958*ros_position,
                                      0.96*ros_position,
                                      0.962*ros_position,
                                      0.964*ros_position,
                                      0.966*ros_position,
                                      0.968*ros_position,
                                      0.97*ros_position,
                                      0.972*ros_position,
                                      0.974*ros_position,
                                      0.976*ros_position,
                                      0.978*ros_position,
                                      0.98*ros_position,
                                      0.982*ros_position,
                                      0.984*ros_position,
                                      0.986*ros_position,
                                      0.988*ros_position,
                                      0.99*ros_position,
                                      0.992*ros_position,
                                      0.994*ros_position,
                                      0.996*ros_position,
                                      0.998*ros_position,
                                      ros_position]
        self.publisher_joint_states.publish(self.joint_states)

    def spin(self):
        while rclpy.ok() and not self.shutdown_requested:

            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_node()

def main(args=None):

    rclpy.init(args=args)

    app = QtWidgets.QApplication([])
    snake_widget = snakeWidgets()

    ros_node = MyNode()
    snake_widget.slider.valueChanged.connect(ros_node.publish_joint_states)

    ros_thread = threading.Thread(target=ros_node.spin)
    ros_thread.start()

    snake_widget.resize(400, 300)
    snake_widget.show()

    result = app.exec_()

    ros_node.join()
    rclpy.shutdown()
    
    sys.exit(result)


if __name__ == '__main__':
    main()
    
