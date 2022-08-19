# BIGSS Snake Tool ROS2 Description

![Demo Video](docs/img/view_snake.png)

Plug in Arduino, and run:

```bash
ros2 run snake_capa_publisher capa_publisher
```

To Calibrate and send the data, run:

```bash
ros2 run snake_capa_publisher capa_mapper
```

To view the snake tool in Rviz2, run:

```bash
ros2 launch snake_description view_snake_tool.launch.py gui:=false
```
