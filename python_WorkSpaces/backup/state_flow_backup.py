import numpy as np
from IMG.get_image import Get_Realsense_Image
from IMG.use_yolo_inference import Use_Yolo
from IMG.get_xyz import Get_XYZ, Get_XYZ_Whole
from IMG.transform_coordinate import Transform_Coordinate
from CONTLROL.control_cart import Plan_Cart_position, Control_Cart
from CONTLROL.control_machine import Plan_Order_Machine_position
from STATEFLOW.state_flow_setup import State_Flow_Setup
import time


def change_node(transition):
    if transition == 'Init_to_Search':
        state_flow.transition(transition)       
        time.sleep(0.1)
        return 'Search'
    elif transition == 'self':
        state_flow.transition(transition)       
        time.sleep(0.1)
        return 'Search'
    elif transition == 'Search_to_Adjustment':
        state_flow.transition(transition)
        time.sleep(0.1)
        return 'Adjustment'
    elif transition == 'Search_to_Finish':
        state_flow.transition(transition) 
        time.sleep(0.1)
        return 'Finish'
    elif transition == 'Adjustment_to_Pollination':
        state_flow.transition(transition)
        time.sleep(0.1)
        return 'Pollination'
    elif transition == 'Adjustment_to_Finish':
        state_flow.transition(transition)  
        time.sleep(0.1)
        return 'Finish'
    elif transition == 'Pollination_to_Origin':
        state_flow.transition(transition)
        time.sleep(0.1)
        return 'Origin'
    elif transition == 'Origin_to_Next':
        state_flow.transition(transition)
        time.sleep(0.1)
        return 'Next'
    elif transition == 'Next_to_Search':
        state_flow.transition(transition)
        time.sleep(0.1)
        return 'Search'
    elif transition == 'Next_to_Adjustment':
        state_flow.transition(transition)        
        time.sleep(0.1)
        return 'Adjustment'
    elif transition == 'Next_to_Finish':
        state_flow.transition(transition)  
        time.sleep(0.1)
        return 'Finish'

def process(node):
    global file_path_color_img, file_path_depth_img, file_path_depth, weight_path, file_path_pixel, threshold
    global height, camera_p_pos, machine_pos, distance

    # Search node
    if node == 'Search':
        # トマト株を微小移動で探索：
        # 10cmずつ探索していく
        move_val = 10
        distance = 90
        Limit = 100

        if distance + move_val > Limit:
            transiton = 'Search_to_Finish'
            return transiton

        # GO(move_val) Arduino制御
        distance += move_val
        # 画像取得し、トマト株を見つける

        '''画像取得'''
        get_img = Get_Realsense_Image()
        get_img.Get_Img(file_path_color_img, file_path_depth_img, file_path_depth)
        ''''''
        '''yolov5による推論'''
        use_yolo = Use_Yolo()
        # file_path_color_img = 'C:/Users/shut1/yolov5/img/color_img.jpg' # dummy
        use_yolo.Execute_Yolo(file_path_color_img, weight_path, threshold)
        ''''''
        '''取得した3次元座標から位置を計算（全体カメラのみ）'''
        get_xyz_whole = Get_XYZ_Whole()
        target_array = get_xyz_whole.Calculate_Center_XYZ(file_path_depth, file_path_pixel)
        ''''''
        '''座標変換'''
        node = Transform_Coordinate(robot_pos, camera_w_pos, camera_w_rotation_matlix, camera_p_rotation_matlix, machine_rotation_matlix, R_w, R_p)
        '''全体カメラ'''
        w = node.transformation_w(target_array) # ロボット座標系からみたトマト株の位置
        ''''''
        '''移動量の計算（全体カメラのみ）'''
        cart = Plan_Cart_position(machine_pos[1], small_move) #  引数はロボット座標系から送風機座標系の基準点までのy方向の距離
        tomato_loc_r, move_val = cart.Identify_Closest_To_Center(w)
        ''''''

        # detect_flag = 0 #ここは消す
        detect_flag = 1 #ここは消す

        if len(tomato_loc_r) == 1:
            detect_flag = 1

        if detect_flag == 1: #トマト株を発見
            transiton = 'Search_to_Adjustment'
            return transiton

        else:
            transiton = 'self'
            # 'self'ならば、GO(move_val)
            distance += move_val
            return transiton
    
    if node == 'Adjustment':

        move_val = 30
        distance = 40
        Limit = 100

        if distance + move_val > Limit:
            transition = 'Adjustment_to_Finish'
            return transition

        else: 
            transition = 'Adjustment_to_Pollination'
            # GO(move_val) Arduino制御
            distance += move_val
            return transition

    if node == 'Pollination':
        # Pollination node
        transition = 'Pollination_to_Origin'
        return transition

    if node == 'Origin':
        # Origin node
        transition = 'Origin_to_Next'
        return transition
    
    if node == 'Next':
        # Next node

        distance = 70
        Limit = 100
        # move_val = Width_steｐ - (disance - tomato_loc)
        move_val = 20

        if distance + move_val > Limit:
            transition = 'Next_to_Finish'
            return transition


        # GO(move_val) Arduino
        distance += move_val
        # 画像取得し、トマト株を見つける

        detect_flag = 0

        if detect_flag == 1: #トマト株を発見
            transiton = 'Next_to_Adjustment'
            return transiton

        else:
            transiton = 'Next_to_Search'
            return transiton
    
    if node == 'Finish':
        move_val = 0
        # GO(move_val) Arduino 制御
        return 'error'

        

# instance
state_flow = State_Flow_Setup()

# Initializeation node
node = 'Initialization'
transiton = 'Init_to_Search'
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
small_move = 10 #微小移動分
state_flow.draw()
state_flow.print_state()

while True:
    node = change_node(transiton)
    state_flow.draw()
    transiton = process(node)
    state_flow.print_state()
