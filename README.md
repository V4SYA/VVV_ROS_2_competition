# VVV_ROS_2_competition

У нас совсем не горят сроки))

## Сборка
colcon build --packages-select autorace_core_exhausted_by_ros autorace_camera

## Выполнение команд
### Запуск мира
ros2 launch robot_bringup autorace_2023.launch.py

### Запуск launch-файла
ros2 launch autorace_core_exhausted_by_ros autorace_core.launch.py

### Запуск referee (трасса оживает)
ros2 run referee_console mission_autorace_2023_referee

Если не запускается, пишите, звоните, трубите во все двери)
