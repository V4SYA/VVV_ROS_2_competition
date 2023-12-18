# VVV_ROS_2_competition

У нас совсем не горят сроки))

## Сборка
colcon build --packages-select autorace_core_vvv referee_console robot_bringup robot_description autorace_camera

## Выполнение команд
### Запуск мира
ros2 launch robot_bringup autorace_2023.launch.py

### Запуск камеры
ros2 launch autorace_camera extrinsic_camera_calibration.launch.py

### Запуск referee (трасса оживает)
ros2 run referee_console mission_autorace_2023_referee

### Запуск launch-файла
ros2 launch autorace_core_vvv detect_lane.launch.py
