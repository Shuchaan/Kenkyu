import numpy as np
import time
from IMG.get_image import Get_Realsense_Image
from IMG.use_yolo_inference import Use_Yolo
from IMG.get_xyz import Get_XYZ, Get_XYZ_Whole
from IMG.transform_coordinate import Transform_Coordinate
from CONTLROL.control_cart import Plan_Cart_position, Control_Cart
from CONTLROL.control_arm import X_Axis_Control, Y_Axis_Control, Z_Axis_Control


height = 0.0 #現在の台座の高さ(cm)
distance = 0.0 #現在のロボットの位置（cm）
file_path_color_img = 'C:/Users/shut1/Desktop/kenkyu/color_img/color_img.jpg'
file_path_depth_img = 'C:/Users/shut1/Desktop/kenkyu/depth_img/depth_img.jpg'
file_path_depth = 'C:/Users/shut1/Desktop/kenkyu/depth/depth.csv'
weight_path = 'C:/Users/shut1/yolov5/best-7-16-2.pt'
threshold = '0.7'
file_path_pixel = 'C:/Users/shut1/Desktop/kenkyu/detect_pixel/color_img.csv'
robot_pos = np.array([0.0, 0.0, 0.0]) # ロボット座標系（基準となる座標系）固定
camera_w_pos = np.array([-7.5, 105.0, 0.0]) # 全体カメラ座標系　固定
camera_p_pos = np.array([-100.0, height, 50.0]) # 手先カメラ座標系　yは変化する。x, zは固定
arm_pos = np.array([64.0, -40.0, 79.5])
machine_pos = np.array([50.0, -90.0, height]) # 送風機座標系 zは変化する。
camera_w_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #全体カメラの回転行列
camera_p_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
machine_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
R_w = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])
R_p = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])

x_control = X_Axis_Control()
y_control = Y_Axis_Control()
z_control = Z_Axis_Control()
use_yolo = Use_Yolo()
get_xyz_whole = Get_XYZ_Whole()
trans = Transform_Coordinate(robot_pos, camera_w_pos, arm_pos, camera_w_rotation_matlix, camera_p_rotation_matlix, machine_rotation_matlix, R_w, R_p)

'''init'''
x_control.move_init()
time.sleep(0.5)
y_control.move_init()
time.sleep(0.5)
z_control.move_init()
time.sleep(0.5)
pre_z = 1.0

'''画像取得'''
Get_Realsense_Image(0).Get_Img(file_path_color_img, file_path_depth_img, file_path_depth)
''''''
'''yolov5による推論'''
use_yolo.Execute_Yolo(file_path_color_img, weight_path, threshold)
''''''
'''取得した3次元座標から位置を計算（全体カメラのみ）'''
target_array = get_xyz_whole.Calculate_Center_XYZ(file_path_depth, file_path_pixel)
print(target_array)
''''''
'''座標変換 全体カメラ'''
a = trans.transformation_a(target_array) # アーム座標系からみたトマト株の位置
print(a)
''''''


for i in range(len(a)):
    x, y, z = a[i]
    print(x, y, z)
    y_control.move_target(y)
    time.sleep(2.0)
    z_control.move_target(z, pre_z)
    pre_z = z
    time.sleep(2.0)
    x_control.move_target(x+3.0)
    time.sleep(5.0)
    x_control.move_target(0.0)
    time.sleep(2.0)


z_control.move_target(5.0, pre_z)
pre_z = 5.0
time.sleep(2.0)
y_control.move_target(10.0)
time.sleep(2.0)

