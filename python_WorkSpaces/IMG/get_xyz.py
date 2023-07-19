import numpy as np
import pandas as pd

class Get_XYZ:
    '''
    input: flie_path_depth csv
           file_path_pixel csv

    output: target_array
    '''

    def __init__(self, pixel_threshold):
        self.WIDTH = 640
        self.HEIGHT = 480
        self.threshold = pixel_threshold
    
    def Identify_Object_Group_in_Four_Region(self, array):
        quarter = [0, 0, 0, 0]
        z_list = [0, 0, 0, 0]
        target_arr = []
        for i in range(len(array)):
            x, y, z = array[i]
            if  0 < x < int(self.WIDTH/2):
                if 0 < y < int(self.HEIGHT/2):
                    quarter[0] += 1
                    z_list[0] += z
                else: #int(self.HEIGHT/2) <= y < self.HEIGHT
                    quarter[1] += 1
                    z_list[1] += z
            else:
                if 0 < y < int(self.WIDTH/2):
                    quarter[2] += 1
                    z_list[2] += z
                else:
                    quarter[3] += 1
                    z_list[3] += z
        
        if quarter[0] > 0:
            target_arr.append([int(self.WIDTH/4)-1, int(self.HEIGHT/4)-1, (z_list[0]/quarter[0])])
        if quarter[1] > 0:
            target_arr.append([int(self.WIDTH/4)-1, int(self.HEIGHT*3/4)-1, (z_list[1]/quarter[1])])
        if quarter[2] > 0:
            target_arr.append([int(self.WIDTH*3/4)-1, int(self.HEIGHT/4)-1, (z_list[2]/quarter[2])])
        if quarter[3] > 0:
            target_arr.append([int(self.WIDTH*3/4)-1, int(self.HEIGHT*3/4)-1, (z_list[3]/quarter[3])])
        
        return target_arr
        
    def Identify_Object_Group(self, a):
        a_copy = a.copy()
        arr_list = []

        while len(a) > 0:
            attention = a[0]
            attention_ind = a_copy.index(attention)
            arr = [attention_ind]

            for i in range(1, len(a)):
                d = (attention[0] - a[i][0])**2 + (attention[1] - a[i][1])**2
                if d <= self.threshold**2:
                    ind = a_copy.index(a[i])
                    arr.append(ind)

            arr_list.append(arr)

            a = [x for x in a if a_copy.index(x) not in arr]

        # 残った最後の要素を追加
        if len(a) > 0:
            arr_list.append([a_copy.index(a[0])])


        target_arr = []
        for i in range(len(arr_list)):
            x = 0
            y = 0
            z = 0
            N = len(arr_list[i])
            for j in range(N):
                ind_num = arr_list[i][j]
                x_, y_, z_ = a_copy[ind_num]
                x += x_ / N
                y += y_ / N
                z += z_ / N
            target_arr.append([int(x), int(y), z])

        return target_arr


    def Calculate_Center_XYZ(self, file_path_depth, file_path_pixel):
        df_depth = pd.read_csv(file_path_depth, header=None)
        df_pixel = pd.read_csv(file_path_pixel)

        N = len(df_pixel)
        array = []

        if N != 0:
            for i in range(int(N)):
                x = df_pixel['x'][i]
                y = df_pixel['y'][i]
                z = df_depth[x][y]*0.1
                array.append([x, y, z])

        # target_arr = self.Identify_Object_Group_in_Four_Region(array)
        target_arr = self.Identify_Object_Group(array)

        return np.array(target_arr)


class Get_XYZ_Whole:
    '''
    input: flie_path_depth csv
           file_path_pixel csv

    output: target_array
    '''

    def __init__(self):
        self.WIDTH = 640
        self.HEIGHT = 480

    def Calculate_Center_XYZ(self, file_path_depth, file_path_pixel):
        df_depth = pd.read_csv(file_path_depth, header=None)
        df_pixel = pd.read_csv(file_path_pixel)

        N = len(df_pixel)
        array = []

        if N != 0:
            for i in range(int(N)):
                x = df_pixel['x'][i]
                y = df_pixel['y'][i]
                z = df_depth[x][y]*0.1
                array.append([x, y, z])

        return np.array(array)

# file_path_depth = 'C:/Users/shut1/Desktop/kenkyu/depth/depth.csv'
# file_path_pixel = 'C:/Users/shut1/Desktop/kenkyu/detect_pixel/color_img.csv'

# get_xyz_whole = Get_XYZ_Whole()
# target_array = get_xyz_whole.Calculate_Center_XYZ(file_path_depth, file_path_pixel)
# print(target_array)
