import serial
import time

# x_axis_armは移動位置＝20-動かしたい位置だけ動かせばいい、アームを初期位置から1cm動かしたいときは、20-1=190.00[mm]
# https://jp.misumi-ec.com/maker/misumi/mech/special/actuator_portal/rs/download/pdf/C1C21C22_JP_V201.pdf
# p230より

class X_Axis_Control:
    """
    input:
        Target Location
    output:
        finish flag
    """
    def __init__(self):
        self.ser = serial.Serial("COM9", baudrate=38400, bytesize=8, parity=serial.PARITY_ODD, stopbits=1, xonxoff=False)

    def servo_on(self):
        self.ser.write(b'@SRVO1,') # サーボON指令
        while True:
            self.ser.write(b'@?OPT1,') # オプション情報（状態）の読み出し
            receive = self.ser.readline()
            if receive == b'OK.1\r\n':
                time.sleep(0.1) # 0.1秒停止
                break

    def servo_off(self):
        self.ser.write(b'@SRVO0,') # サーボOFF指令
        time.sleep(0.1)

    def org_arm(self):
        self.ser.write(b'@ORG,') # 限定回帰指令
        while True:
            self.ser.write(b'@?OPT1,') # オプション情報（状態）の読み出し
            receive = self.ser.readline()
            # 2584, 2508, 2568 が終了応答
            if receive == b'OPT1.1=2584\r\n' or receive == b'OPT1.1=2508\r\n' or receive == b'OPT1.1=2568\r\n':
                time.sleep(0.1) # 0.1秒停止
                break
    
    def move_init(self):
        self.servo_on()
        self.org_arm()
        self.move_target(0.0)
        return 0

    def move_target(self, target):
        if target < 0:
            target = 0.0
        if target > 20.0:
            target = 20.0
        target = str(int(round(target, 1)*10)) + "00"
        target = 20000 - int(target)
        self.ser.write(('@S1=50,').encode(encoding='utf-8')) # Speedの設定
        time.sleep(0.1) # 0.1秒停止
        self.ser.write(('@START1#P'+str(target)+',').encode(encoding='utf-8')) #目標位置へ移動指令
        while True:
            self.ser.write(b'@?OPT1,') # オプション情報（状態）の読み出し
            receive = self.ser.readline()
            # 2570, 2346が終了応答
            if receive == b'OPT1.1=2570\r\n' or receive == b'OPT1.1=2346\r\n':
                time.sleep(0.1) # 0.1秒停止
                break
        return 0

class Y_Axis_Control:
    """
    input:
        Target Location
    output:
        finish flag
    """
    def __init__(self):
        self.ser = serial.Serial("COM8", baudrate=38400, bytesize=8, parity=serial.PARITY_ODD, stopbits=1, xonxoff=False)

    def servo_on(self):
        self.ser.write(b'@SRVO1,') # サーボON指令
        while True:
            self.ser.write(b'@?OPT1,') # オプション情報（状態）の読み出し
            receive = self.ser.readline()
            if receive == b'OK.1\r\n':
                time.sleep(0.1) # 0.1秒停止
                break

    def servo_off(self):
        self.ser.write(b'@SRVO0,') # サーボOFF指令
        time.sleep(0.1)

    def org_arm(self):
        self.ser.write(b'@ORG,') # 限定回帰指令
        while True:
            self.ser.write(b'@?OPT1,') # オプション情報（状態）の読み出し
            receive = self.ser.readline()
            # 2584, 2508, 2568 が終了応答
            if receive == b'OPT1.1=2584\r\n' or receive == b'OPT1.1=2508\r\n' or receive == b'OPT1.1=2568\r\n':
                time.sleep(0.1) # 0.1秒停止
                break
    
    def move_init(self):
        self.servo_on()
        self.org_arm()
        self.move_target(10.0)
        return 0

    def move_target(self, target):
        if target < 0:
            target = 0.0
        if target > 20.0:
            target = 20.0
        target = str(int(round(target, 1)*10)) + "00"
        self.ser.write(('@S1=50,').encode(encoding='utf-8')) # Speedの設定
        time.sleep(0.1) # 0.1秒停止
        self.ser.write(('@START1#P'+str(target)+',').encode(encoding='utf-8')) #目標位置へ移動指令
        while True:
            self.ser.write(b'@?OPT1,') # オプション情報（状態）の読み出し
            receive = self.ser.readline()
            # 2570, 2346が終了応答
            if receive == b'OPT1.1=2570\r\n' or receive == b'OPT1.1=2346\r\n':
                time.sleep(0.1) # 0.1秒停止
                break
        return 0

class Z_Axis_Control:

        """
        input:
                send_data 
                    -send_data[0]: 'I'→Initialize_mode 'T'→Target_mode
                    -send_data[1]: アーム移動位置の10の位
                    -send_data[2]: アーム移動位置の1の位
                    -send_data[3]: アーム移動位置の1/10の位
                    -send_data[4]: 前回のアームの位置の10の位
                    -send_data[5]: 前回のアームの位置の1の位
                    -send_data[6]: 前回のアームの位置の1/10の位
                    -send_data[7]: 入力終了判定の','
                
                (example1) send_data = 'T105090,' 目標位置を10.5cm、前回の位置は9.0cm
                (example1) send_data = 'T200010,' 目標位置を20.0cm、前回の位置は1.0cm

        output:
                receive_data
                    -finish flag

        """

        def __init__(self):
            self.ser = serial.Serial('COM13', 115200)
        
        def move_init(self):

            target = 1.0
            target = format(int(round(target, 1)*10), '03')

            send_data = 'I' + target + '000'
            send_data = self.check_data(send_data)
            time.sleep(2) # 1秒停止する。1秒でないとうまく動かなかった
            self.ser.write(send_data.encode(encoding='utf-8'))
            receive_data = self.serial_data()
            return 0

        def move_target(self, target, pre_target):

            if target < 1.0:
                target = 1.0
            if target > 40.0:
                target = 40.0

            target = format(int(round(target, 1)*10), '03')
            pre_target = format(int(round(pre_target, 1)*10), '03')

            send_data = 'T' + target + pre_target
            send_data = self.check_data(send_data)
            time.sleep(2) # 1秒停止する。1秒でないとうまく動かなかった
            self.ser.write(send_data.encode(encoding='utf-8'))
            receive_data = self.serial_data()
            return 0

        def check_data(self, send_data):
            # 40.0cmより上にはいかないようにする。
            num = int(send_data[1])*10+int(send_data[2])*1+int(send_data[3])*0.1
            if num > 40.0:
                send_data = send_data[0] + '400'

            # もし語尾に','がなければ追加する 忘れ防止のため
            if send_data[-1] != ',':
                send_data = send_data + ','
            return send_data

        def serial_data(self):
            line = self.ser.readline()
            line_disp = line.strip().decode('UTF-8')
            return line_disp 

'''instance'''
# x_control = X_Axis_Control()
# y_control = Y_Axis_Control()
# z_control = Z_Axis_Control()
'''init'''
# x_control.move_init()
# time.sleep(0.5)
# y_control.move_init()
# time.sleep(0.5)
# z_control.move_init()
# time.sleep(0.5)
'''target'''
# x_control.move_target(10.0)
# time.sleep(0.5)
# y_control.move_target(10.0)
# time.sleep(0.5)
# pre_z = 1.0
# target_z = 5.0
# z_control.move_target(target_z, pre_z)
# time.sleep(0.5)
# pre_z = target_z