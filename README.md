#Для чтения двух байт из serial порта можно использовать UART преобразователь. Код на компьютере будет выглядеть так:

import serial
import time
ser = serial.Serial("/dev/ttyUSB0", 115200)

# ser.open()
while not ser.is_open:
    pass
print("open")
while True:
    ser.write(b'BBBB')
    data = ser.read(61)
    number = int.from_bytes(data, "big", signed=True)
    print(number)
    time.sleep(1)
