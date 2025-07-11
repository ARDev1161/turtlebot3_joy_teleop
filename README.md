# turtlebot3_joy_teleop

C++ ROS 2 пакет для фонового управления TurtleBot3 с помощью джойстика или 3D-мыши (spacenavd).

---

## Содержание

- [Установка](#Установка)
- [Сборка](#Сборка)
- [Запуск](#Запуск)
- [Параметры](#Параметры)
- [Структура пакета](#Структура-пакета)
- [Лицензия](#Лицензия)

---

## Установка

1. Склонируйте репозиторий в папку `src` вашего ROS 2 workspace:
    ```bash
    cd ~/turtlebot3_ws/src
    git clone <URL_репозитория> turtlebot3_joy_teleop
    ```
2. Установите зависимости:
    ```bash
    sudo apt update
    sudo apt install ros-jazzy-joy ros-jazzy-spacenav-node
    ```

## Сборка

В корне workspace выполните:

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
```

После успешной сборки не забудьте источить окружение:

```bash
source install/setup.bash
```

## Запуск

Запустите всё одной командой через launch-файл:

```bash
ros2 launch turtlebot3_joy_teleop teleop.launch.py
```

Должны стартовать три ноды:

- `joy_node` — чтение джойстика (`/dev/input/js0`)
- `spacenav_node` — чтение 3D-мыши от spacenavd
- `teleop_node` — преобразование `/joy` и `/spacenav/joy` в команды `/cmd_vel`

## Параметры

| Параметр         | Тип     | По умолчанию | Описание                                            |
|------------------|---------|--------------|-----------------------------------------------------|
| enable_button    | integer | 5            | Номер кнопки для активации управления (0-based)     |
| axis_linear      | integer | 1            | Ось для линейной скорости                          |
| axis_angular     | integer | 0            | Ось для угловой скорости                           |
| scale_linear     | double  | 0.5          | Коэффициент линейной скорости                      |
| scale_angular    | double  | 0.5          | Коэффициент угловой скорости                       |
| require_enable   | boolean | true         | Требовать нажатие `enable_button` для публикации   |

Переопределение через launch или `--ros-args`:  
```bash
ros2 run turtlebot3_joy_teleop teleop_node   --ros-args -p require_enable:=false -p scale_linear:=1.0
```

## Структура пакета

```
turtlebot3_joy_teleop_cpp/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── teleop.launch.py
└── src/
    └── teleop_node.cpp
```

## Лицензия

Apache License 2.0
