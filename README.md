# BIGSS Snake Tool ROS2 Description

![Demo Video](docs/img/view_snake.png)

To view the snake tool in Rviz2, run:
```bash
ros2 launch snake_description view_snake_tool.launch.py
```
To use obtain the data for simulation, run:
```bash
ros2 run py_pubsub talker
```
To Calibrate and send the data, run:
```bash
ros2 run snake_description snake_bend_capa_mapper
```
