## 1. Установка лаб

Открываем терминал, переходим в директорию, где у вас будет лежать workspace директория с лабами. В терминале пишем
```bash
$ git clone https://github.com/BJLADYA/POUK.git ros2_ws
```

Теперь у вас должна появиться директория `ros2_ws`
```
ros2_ws
├── README.md
└── src
    ├── lab1
    ├── lab2
    ├── lab3
    ├── lab4
    ├── lab5
    └── lab6
```

## 2. Устанавливаем зависимости
```bash
$ sudo apt update && sudo apt upgrade -y
$ sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-nav2-msgs
$ sudo apt install ros-humble-slam-toolbox
```

## 3. Устанавливаем stage
Открываем терминал, переходим в директорию `ros2_ws/src`
```bash
$ git clone https://github.com/tuw-robotics/stage_ros2.git
$ git clone https://github.com/tuw-robotics/Stage.git
```
Переходим в директорию `ros2_ws` и билдим пакеты
```bash
$ cd ..
$ colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY --packages-select Stage
$ colcon build --symlink-install --packages-select stage_ros2
```

## 4. Билдим
В директории `ros2_ws` открываем терминал и пишем
```bash
$ colcon build --symlink-install
```
