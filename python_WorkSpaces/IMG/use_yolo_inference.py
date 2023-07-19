import subprocess
import glob
import os
import pandas as pd

class Use_Yolo:
    '''
    input: image path
           weight path
           threshold


    output: detect_img                    jpg
            detect_pixel(x axis, y axis)  csv
    '''

    def __init__(self):
        self.WIDTH = 640
        self.HEIGHT = 480
        self.Pixcel_List_Path = 'C:/Users/shut1/Desktop/kenkyu/detect_pixel/color_img.csv'
    
    def Execute_Yolo(self, image_path, weight_path, threshold):

        subprocess.run(['C:/Python37/python.exe', 
         'C:/Users/shut1/yolov5/detect.py',
         '--source', image_path,
         '--weight', weight_path,
         '--conf',  threshold,
         '--save-txt',
        ])

        df = pd.read_csv(self.Pixcel_List_Path)
        df['x'] = df['x'] * self.WIDTH
        df['y'] = df['y'] * self.HEIGHT
        df['x'] = df['x'].astype('int')
        df['y'] = df['y'].astype('int')
        df = df.drop(columns=['cls', 'w', 'h', 'null'])


        with open(self.Pixcel_List_Path, 'a', newline='')as f:
            f.truncate(0) #現在のファイルサイズを０にする

        df.to_csv(self.Pixcel_List_Path, index=False, header=True)


# image_path = 'C:/Users/shut1/Desktop/kenkyu/color_img/color_img.jpg'
# weight_path = 'C:/Users/shut1/yolov5/best_4_16_3.pt'
# threshold = '0.7'

# a = Use_Yolo()
# a.Execute_Yolo(image_path, weight_path, threshold)