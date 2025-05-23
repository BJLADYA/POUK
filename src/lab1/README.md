# Запуск

1. Открываем терминал, переходим в `ros2_ws` директорию, в терминале прописываем

```bash
$ source install/setup.bash
```

2. Запуск ноды `listener`
```bash
$ ros2 run lab1 listener
```

3. Открываем другой терминал, повторяем первый шаг. Запускаем ноду `talker`
```bash
$ ros2 run lab1 talker
```