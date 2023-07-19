import serial
import time

class Main_Switch_Control:

        """
        実行するだけでok
        """

        def __init__(self):
            self.ser = serial.Serial('COM7', 115200)
        
        def switch_on(self):
            send_data = "255"
            time.sleep(2) # 1秒停止する。1秒でないとうまく動かなかった
            self.ser.write(send_data.encode(encoding='utf-8'))
            # receive_data = self.serial_data()
            return 0

# sw = Main_Switch_Control()
# sw.switch_on()

# print(int('11010100', 2))
# print(int('11111110', 2))