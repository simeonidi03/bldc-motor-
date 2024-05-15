#Для чтения двух байт из serial порта можно использовать UART преобразователь. Код на компьютере будет выглядеть так:

import serial
import time

# Инициализация последовательного порта
ser = serial.Serial("/dev/ttyUSB0", 115200)

# Ждем, пока порт откроется
while not ser.is_open:
    pass

print("Serial port is open")

# Бесконечный цикл приема данных
while True:
    # Отправка данных
    ser.write(b'BBBB')

    # Переменная для хранения принятой строки
    received_string = ""

    # Считывание данных посимвольно, пока не встретится символ новой строки
    while True:
        # Чтение одного символа из порта
        char = ser.read().decode('utf-8')

        # Добавление символа к строке
        received_string += char

        # Если встречен символ новой строки, завершаем чтение
        if char == '\n':
            break

    # Удаление символа новой строки из строки, если он присутствует
    received_string = received_string.strip()

    # Вывод принятой строки
    print("Received:", received_string)

    # Пауза перед отправкой следующей порции данных
    time.sleep(1)
