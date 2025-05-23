# Запуск

1. Открываем терминал, переходим в `ros2_ws` директорию, в терминале прописываем

```bash
$ source install/setup.bash
```

2. Запуск `stage`
```bash
$ ros2 launch lab2 start.launch.py
```

3. Открываем другой терминал, повторяем первый шаг. Запускаем алгоритм 1 или 2
```bash
$ ros2 run lab2 alg1
или
$ ros2 run lab2 alg2
```