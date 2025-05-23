# Запуск

1. Открываем терминал, переходим в `ros2_ws` директорию, в терминале прописываем

```bash
$ source install/setup.bash
```

2. Запуск `stage`
```bash
$ ros2 launch lab4 start.launch.py
```

3. Открываем еще один терминал, выполняем пункт 1

4. Запускаем ноду для выбора контроллера
```bash
$ ros2 run lab4 selector_node
```