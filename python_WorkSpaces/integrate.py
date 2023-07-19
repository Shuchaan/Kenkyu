import numpy as np
from IMG.get_image import Get_Realsense_Image
from IMG.use_yolo_inference import Use_Yolo
from IMG.get_xyz import Get_XYZ
from IMG.transform_coordinate import Transform_Coordinate
from CONTLROL.control_cart import Plan_Cart_position, Control_Cart
from CONTLROL.control_machine import Plan_Order_Machine_position

'''zzzzzzzzzz
ロボット座標系・・・ロボットに固定された座標系。
1. 全体カメラによって、Color画像とDepth画像とDepth情報を取得する。 ok
2. Yolov5を実行し、画像におけるトマト株の中心位置を取得する（複数取得された場合、x座標が最も中心にあるものを採用）。 ok
3. 着目したトマト株を画像座標系→全体カメラ座標系→ロボット座標系へ変換し、トマト株の中心へ送風機が位置するための移動量を計算する。 ok


1. 手先カメラによって、Color画像とDepth画像とDepth情報を取得する。 ok
2. Yolov5を実行し、画像における花の中心位置を取得する。 ok
3. Depth情報と中心位置を組み合わせて、認識したトマト花すべての3次元位置を取得する。 ok
4. トマト花の3次元位置にすべて風を当てるのは効率が悪いので、まとめて風を送る座標を計算する。 ok
5. 上で計算したものは画像座標系における送風位置である。
   送風位置を画像座標系→手先カメラ座標系→ロボット座標系→送風機座標系へ変換し、送風機座標系における送風位置を計算する。ok
'''

'''init'''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''
file_path_color_img = 'C:/Users/shut1/Desktop/kenkyu/color_img/color_img.jpg'
file_path_depth_img = 'C:/Users/shut1/Desktop/kenkyu/depth_img/depth_img.jpg'
file_path_depth = 'C:/Users/shut1/Desktop/kenkyu/depth/depth.csv'
weight_path = 'C:/Users/shut1/yolov5/best_4_16_3.pt'
threshold = '0.7'
file_path_pixel = 'C:/Users/shut1/Desktop/kenkyu/detect_pixel/color_img.csv'

pixel_threshold = 50 # まとまりとしてみなす閾値 (pixel)

distance = 0.0 #現在のロボットの位置（cm）
height = 0.0 #現在の台座の高さ(cm)
count = 1 #繰り返し数
robot_pos = np.array([0.0, 0.0, 0.0]) # ロボット座標系（基準となる座標系）固定
camera_w_pos = np.array([-5.0, 100.0, 0.0]) # 全体カメラ座標系　固定
camera_p_pos = np.array([-100.0, height, 50.0]) # 全体カメラ座標系　yは変化する。x, zは固定
machine_pos = np.array([50.0, -90.0, height]) # 送風機座標系 zは変化する。
camera_w_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #全体カメラの回転行列
camera_p_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
machine_rotation_matlix =  np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
R_w = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])
R_p = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])

Limit_height = 160 # 台座の上昇高さの限界（cm）
Limit_Rail = 300 # レールの長さの限界（cm）
Height_step = 46.54 #高さ方向の画角分 60cm離れているときは46.54(cm)
Width_step = 60 #トマト株の間隔
small_move = 10
''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''''

'''画像取得'''
file_path_color_img = 'C:/Users/shut1/Desktop/kenkyu/color_img/color_img.jpg'
get_img = Get_Realsense_Image()
get_img.Get_Img(file_path_color_img, file_path_depth_img, file_path_depth)
''''''

'''yolov5による推論'''
use_yolo = Use_Yolo()
file_path_color_img = 'C:/Users/shut1/yolov5/color_img.jpg' # dummy
use_yolo.Execute_Yolo(file_path_color_img, weight_path, threshold)
''''''

'''取得した3次元座標から送風する位置を計算（手先カメラのみ）'''
get_xyz = Get_XYZ(pixel_threshold)
target_array = get_xyz.Calculate_Center_XYZ(file_path_depth, file_path_pixel)
print('pixel\n', target_array)
''''''

'''座標変換'''
node = Transform_Coordinate(robot_pos, camera_w_pos, camera_w_rotation_matlix, camera_p_rotation_matlix, machine_rotation_matlix, R_w, R_p)
'''全体カメラ'''
w = node.transformation_w(target_array) # ロボット座標系からみたトマト株の位置
print('real\n', w)
'''手先カメラ'''
# p = node.transformation_p(target_array, camera_p_pos, machine_pos) #手先カメラ
# print('real\n', p)
''''''

'''移動量の計算（全体カメラのみ）'''
cart = Plan_Cart_position(machine_pos[1], small_move) #  引数はロボット座標系から送風機座標系の基準点までのy方向の距離
tomato_loc_r, move_val = cart.Identify_Closest_To_Center(w)
print(move_val)
''''''

'''
ここにカートを動かすコードを書く
Limit_Rail
distance 　　　　移動距離[cm]（絶対座標系）
move_val 　　　  ロボットの移動量[cm]
tomato_loc            トマト株の位置[cm]（絶対座標系）
tomato_loc_r        トマト株の位置[cm]（ロボット座標系）
GO(move_val)       ロボットを移動させる関数。引数はmove_val

node 1
トマト株を微小移動で探索：
10cmずつ探索していく
move_val = 10 
GO(move_val)
distance += move_val
画像取得し、トマト株を見つける

node 2
tomato_loc_rを取得　tomato_loc_r
tomato_loc = distance + tomato_loc_r
最も中心に近いトマト株が送風機の中心に位置するための(move_val)を取得　move_val = cart.Identify_Closest_To_Center(w)
GO(move_val)
distance += move_val

node 3
move_val = Width_steｐ - (disance - tomato_loc)
GO(move_val)
distance += move_val
画像処理し、トマト株を見つける

node 4
Limit_Railを超えてしまうなら、STOP

'''
'''送風順番（下から上）の決定（手先カメラのみ）'''
# machine = Plan_Order_Machine_position(Limit_height_height, Height_step)
# array = np.array([[10, 0., 10], [-10, 2, -10], [-10, -1, 20], [-10,40, 20]])
# arr = machine.Identify_Order_Movement(height, array)
# print(arr)
''''''

'''
ここに昇降部を動かすコードを書く
目標位置へ噴射
高さの更新:height += Height_step
'''

'''
一連の動作終了後カートとアームを指定の位置へ動かすコードを書く
height = 0
移動量:move_val = Width_step*count - distance
移動後:distance = Width_step*count
count += 1
'''
