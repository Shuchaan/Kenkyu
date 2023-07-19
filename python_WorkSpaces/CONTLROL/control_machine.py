import numpy as np

class Plan_Order_Machine_position:
    '''
    もし上位制御をmatlabでやるならここはいらないのかもしれない。
    なぜなら、現在の台座の位置は別のノード（Sim）から受け取る必要があるから
    送風順番は、y軸の値が小さい順にしている。

    input: 
            台座の高さの限界値 Limit(cm) 固定値
            現在の台座の高さ height(cm) 変動値

    output:
            ロボット座標系からみてy軸方向の絶対値の最も小さいトマト株を特定し、
            送風機の中心にそのトマト株が位置するような移動量（cm）

    '''
    def __init__(self, Limit, Height_step):
        self.Limit = Limit
        self.Height_step = Height_step

    def Identify_Order_Movement(self, height, array):
        if len(array) == 0:
            return array
        sorted_coordinates = array[array[:, 1].argsort()]
        target_array = self.If_Possible_Or_Impossible(self.Limit, self.Height_step, height, array)
        return target_array
    
    def If_Possible_Or_Impossible(self, Limit, Height_step, height, array):
        coordinates = []
        if height == 0.0: # 台座が初期位置にあるときは、それより下には移動しない
            for i in range(len(array)):
                if array[i][1] >= 0:
                    coordinates.append(array[i])
            return np.array(coordinates)

        elif 0 <= Limit - height < Height_step: #  限界-現在の位置の差が、画角より小さいとき、リミットスイッチより上には移動しない
            for i in range(len(array)):
                x, y, z = array[i]
                if Limit - height >= y:
                    coordinates.append(array[i])
            return np.array(coordinates)

        elif Limit - height >= Height_step: # まだまだ考慮しなくてよい
            return array
        
        else: #  万が一のため移動させない
            return np.array(coordinates)

# machine = Plan_Order_Machine_position(160, 40)
# height = 0.0
# array = np.array([[10, 0., 10]])
# array = np.array([])
# arr = machine.Identify_Order_Movement(height, array)
# print(arr)