import numpy as np
import serial
import time

class Plan_Cart_position:
    '''
    input: 
            ロボット座標系から送風機座標系の基準点までのy方向の距離L
            ロボット座標系からみたトマト株の位置（x, y, z）

    output:
            ロボット座標系からみてy軸方向の絶対値の最も小さいトマト株を特定し、
            送風機の中心にそのトマト株が位置するような移動量（cm）
            ロボット座標系からみたそのトマト株の位置

    '''
    def __init__(self, L, small_move):
        self.L = L
        self.small_move = small_move

    def Determine_Move_Value(self, val):
        move_val = val - self.L
        return move_val
    
    def Identify_Closest_To_Center(self, array):
        if len(array) == 0:
            return [], self.small_move

        min_abs_value = np.min(np.abs(array[:, 1]))
        min_abs_index = np.where(np.abs(array[:, 1]) == min_abs_value)[0]
        min_val = array[int(min_abs_index)][1]
        move_val = self.Determine_Move_Value(min_val)
        return array[min_abs_index][0][1], int(move_val)

# L = -90 # ロボット座標系から送風機座標系の基準点までのy方向の距離
# array = np.array([[100, -5, 30], [150, 2, 3], [2, 30, -1]])
# node = Plan_Cart_position(L)
# tomato_loc_r, val = node.Identify_Closest_To_Center(array)
# print(tomato_loc_r)
# print(val)

class Control_Cart:
    '''
    Arduinoによるカート制御
    このクラスへの入力は、移動量のみを入力してください。
    例）move_val = 100　→　正方向へ10.0cm移動します

    To Arduino
    input:  move_val:  ロボットの移動量[cm] 
            F:         Forward 正転　進行方向
            R:         Reverse 逆転  逆方向
    output:
            Finish_flag 正常終了 0

    '''
    def __init__(self):
        self.ser = serial.Serial('COM3', 115200)
    
    def Go(self, move_val):
        # 進行方向へ
        if move_val > 0:
            send_data = format(int(round(move_val, 1)*10), '04')
            send_data = 'F' + send_data
        # 逆方向へ
        elif move_val < 0:
            send_data = format(int(round(move_val, 1)*10), '05')
            send_data = 'R' + send_data[1:]
        # 停止
        else:
            send_data = 'F0000'

        time.sleep(2)
        send_data = self.check_data(send_data)
        self.ser.write(send_data.encode(encoding='utf-8'))
        receive_data = self.serial_data() #何か文字を受け取ったら終了
        # print(receive_data)
        return 0
    
    def check_data(self, send_data):
        if send_data[-1] != ',':
            send_data = send_data + ','
        return send_data

    def serial_data(self):
        line = self.ser.readline()
        line_disp = line.strip().decode('UTF-8')
        return line_disp

# cart_control = Control_Cart()
# cart_control.Go(30.0)
# cart_control.Go(-30.0)