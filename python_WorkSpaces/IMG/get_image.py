import pyrealsense2 as rs
import cv2
import numpy as np
import os
import csv
import shutil

class Get_Realsense_Image:
    '''
    input: 
            device_num
            file_path
    output:
            color_img  jpg
            depth_img  jpg
            depth_data csv
    '''

    def __init__(self, device_num):
        self.WIDTH = 640
        self.HEIGHT = 480
        # ストリーム(Color/Depth)の設定
        self.config = rs.config()
        if device_num == 0:
            # 全体カメラ
            self.config.enable_device('814412070380')
        elif device_num == 1:
            # 手先カメラ
            self.config.enable_device('912112072691')
        
        #IRカメラ1の設定
        self.config.enable_stream(rs.stream.infrared, 1, self.WIDTH, self.HEIGHT, rs.format.y8, 30)
        #IRカメラ2の設定
        self.config.enable_stream(rs.stream.infrared, 2, self.WIDTH, self.HEIGHT, rs.format.y8, 30)
        # Colarカメラの設定
        self.config.enable_stream(rs.stream.color, self.WIDTH, self.HEIGHT, rs.format.bgr8, 30)
        # Depthカメラの設定
        self.config.enable_stream(rs.stream.depth, self.WIDTH, self.HEIGHT, rs.format.z16, 30)

        # ストリーミング開始
        self.pipeline = rs.pipeline()
        self.profile = self.pipeline.start(self.config)

        # Alignオブジェクト生成
        self.align_to = rs.stream.color
        # 深度画像とカラー画像の位置合わせを行う
        self.align = rs.align(self.align_to)

        self.depth_intrinsics = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth)).get_intrinsics()
        self.color_intrinsics = rs.video_stream_profile(self.profile.get_stream(rs.stream.color)).get_intrinsics()

    def Get_Camera_Intrinsics(self):
         # [618.6954956054688, 618.8485717773438, 320.0, 240.0]
        R=[self.color_intrinsics.fx,
           self.color_intrinsics.fy,
           self.WIDTH/2,
           self.HEIGHT/2
        ]
        return R

    
    def Duplicate_Rename(self, file_path):
        if os.path.exists(file_path):
            name, ext = os.path.splitext(file_path)
            i = 1
            while True:
                # 数値を3桁などにしたい場合は({:0=3})とする
                new_name = "{}({}){}".format(name, i, ext)
                if not os.path.exists(new_name):
                    return new_name
                i += 1
        else:
            return file_path

    def Move_File(self, old, new):
        try:
            shutil.move(old, new)
        except FileNotFoundError:
            a = ''

    def Write_Depth_Info(self, file_path, depth_image):
        with open(file_path, 'w') as f:
            writer = csv.writer(f)
            writer = csv.writer(f, lineterminator='\n')
            writer.writerows(depth_image)

    def Get_Img(self, file_path_color_img, file_path_depth_img, file_path_depth):
        new_file_path_color_img = self.Move_File(file_path_color_img, self.Duplicate_Rename(file_path_color_img))
        new_file_path_depth_img = self.Move_File(file_path_depth_img, self.Duplicate_Rename(file_path_depth_img))
        new_file_path_depth = self.Move_File(file_path_depth, self.Duplicate_Rename(file_path_depth))

        # RealSenseのRGBカメラは、立ち上げた直後だと少し赤みがかった色を取得してきてしまうので、
        # しばらくしてRGBがきちんと取得できるようになるまでforループを回しています。
        for i in range(30):
            fs = self.pipeline.wait_for_frames()
        
        """get_color_and_depth image"""
        # フレームのセットを取得．
        aligned_frames = self.align.process(fs)
        # フレームのカラー画像を取得．
        color_frame = aligned_frames.get_color_frame()
        # フレームのデプス画像を取得．
        depth_frame = aligned_frames.get_depth_frame()
        # ストリーミング停止.
        self.pipeline.stop()
        # imageをnumpy配列に変換
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # depth imageをカラーマップに変換
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.2), cv2.COLORMAP_JET)

        # 保存
        cv2.imwrite(file_path_color_img, color_image)
        cv2.imwrite(file_path_depth_img, depth_colormap)
        self.Write_Depth_Info(file_path_depth, depth_image)
