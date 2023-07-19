import numpy as np
from IMG.get_image import Get_Realsense_Image
from IMG.use_yolo_inference import Use_Yolo
from IMG.get_xyz import Get_XYZ, Get_XYZ_Whole
from IMG.transform_coordinate import Transform_Coordinate
from CONTLROL.control_cart import Plan_Cart_position, Control_Cart
from CONTLROL.control_machine import Plan_Order_Machine_position
from STATEFLOW.state_flow_setup import State_Flow_Setup
import time

class STATEMACHINE:
    def __init__(self):
        self.node = 'Initialization'
        self.transiton = 'Init_to_Search'
        self.file_path_color_img = 'C:/Users/shut1/Desktop/kenkyu/color_img/color_img.jpg'
        self.file_path_depth_img = 'C:/Users/shut1/Desktop/kenkyu/depth_img/depth_img.jpg'
        self.file_path_depth = 'C:/Users/shut1/Desktop/kenkyu/depth/depth.csv'
        self.weight_path = 'C:/Users/shut1/yolov5/best_4_16_3.pt'
        self.threshold = '0.7'
        self.file_path_pixel = 'C:/Users/shut1/Desktop/kenkyu/detect_pixel/color_img.csv'
        self.pixel_threshold = 50 # まとまりとしてみなす閾値 (pixel)
        self.robot_pos = np.array([0.0, 0.0, 0.0]) # ロボット座標系（基準となる座標系）固定
        self.camera_w_pos = np.array([-5.0, 100.0, 0.0]) # 全体カメラ座標系　固定
        self.camera_w_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #全体カメラの回転行列
        self.camera_p_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
        self.machine_rotation_matlix =  np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
        self.R_w = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])
        self.R_p = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])
        self.Limit_height = 160 # 台座の上昇高さの限界（cm）
        self.Limit_Rail = 300 # レールの長さの限界（cm）
        self.Height_step = 46.54 #高さ方向の画角分 60cm離れているときは46.54(cm)
        self.Width_step = 60 #トマト株の間隔
        self.small_move = 10 #微小移動分
        self.machine_pos_Length = -90.0


        '''instance'''
        self.use_yolo = Use_Yolo()
        self.get_xyz_whole = Get_XYZ_Whole()
        self.trans = Transform_Coordinate(self.robot_pos, self.camera_w_pos, self.camera_w_rotation_matlix, self.camera_p_rotation_matlix, self.machine_rotation_matlix, self.R_w, self.R_p)
        self.cart = Plan_Cart_position(self.machine_pos_Length, self.small_move)
        # self.cart_control = Control_Cart()

    def change_node(self, transition):
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

    def process(self, node):
        global distance, height, count, move_val
        camera_p_pos = camera_p_pos = np.array([-100.0, height, 50.0]) # 全体カメラ座標系　yは変化する。x, zは固定
        machine_pos = np.array([50.0, -90.0, height]) # 送風機座標系 zは変化する。

        # Search node
        if node == 'Search':
            # トマト株を微小移動で探索：
            # 10cmずつ探索していく
            move_val = self.small_move

            if distance + move_val > self.Limit_Rail:
                transiton = 'Search_to_Finish'
                return transiton
            
            '''移動'''
            # self.cart_control.Go(move_val)
            distance += move_val


            '''画像取得'''
            Get_Realsense_Image().Get_Img(self.file_path_color_img, self.file_path_depth_img, self.file_path_depth)
            ''''''
            '''yolov5による推論'''
            self.use_yolo.Execute_Yolo(self.file_path_color_img, self.weight_path, self.threshold)
            ''''''
            '''取得した3次元座標から位置を計算（全体カメラのみ）'''
            target_array = self.get_xyz_whole.Calculate_Center_XYZ(self.file_path_depth, self.file_path_pixel)
            ''''''
            '''座標変換 全体カメラ'''
            w = self.trans.transformation_w(target_array) # ロボット座標系からみたトマト株の位置
            ''''''
            '''移動量の計算（全体カメラのみ）'''
            tomato_loc_r, move_val = self.cart.Identify_Closest_To_Center(w)
            ''''''

            # detect_flag = 0 #ここは消す
            detect_flag = 1 #ここは消す

            if len(tomato_loc_r) == 1:
                detect_flag = 1

            if detect_flag == 1: #トマト株を発見
                transiton = 'Search_to_Adjustment'
                count += 1
                return transiton

            else:
                transiton = 'self'
                return transiton
        
        if node == 'Adjustment':

            move_val = 20 # ここは消す

            if distance + move_val > self.Limit_Rail:
                transition = 'Adjustment_to_Finish'
                return transition

            else: 
                transition = 'Adjustment_to_Pollination'
                '''移動'''
                # self.cart_control.Go(move_val)
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
            # move_val = self.Width_step - (disance - tomato_loc)
            move_val = 30 #ここは消す

            if distance + move_val > self.Limit_Rail:
                transition = 'Next_to_Finish'
                return transition

            '''移動'''
            # self.cart_control.Go(move_val)
            distance += move_val

            '''画像取得'''
            Get_Realsense_Image().Get_Img(self.file_path_color_img, self.file_path_depth_img, self.file_path_depth)
            ''''''
            '''yolov5による推論'''
            self.use_yolo.Execute_Yolo(self.file_path_color_img, self.weight_path, self.threshold)
            ''''''
            '''取得した3次元座標から位置を計算（全体カメラのみ）'''
            target_array = self.get_xyz_whole.Calculate_Center_XYZ(self.file_path_depth, self.file_path_pixel)
            ''''''
            '''座標変換 全体カメラ'''
            w = self.trans.transformation_w(target_array) # ロボット座標系からみたトマト株の位置
            ''''''
            '''移動量の計算（全体カメラのみ）'''
            tomato_loc_r, move_val = self.cart.Identify_Closest_To_Center(w)
            ''''''

            if len(tomato_loc_r) == 1:
                detect_flag = 1

            detect_flag = 0

            if detect_flag == 1: #トマト株を発見
                count += 1
                transiton = 'Next_to_Adjustment'
                return transiton

            else:
                transiton = 'Next_to_Search'
                return transiton
        
        if node == 'Finish':
            move_val = 0
            # self.cart_control.Go(move_val)
            return 'error'
    
    def start(self):
        node = 'Initialization'
        transiton = 'Init_to_Search'
        state_flow.draw()
        state_flow.print_state()
        print('distance', distance)
        # print('height', height)

        while True:
            node = self.change_node(transiton)
            state_flow.draw()
            transiton = self.process(node)
            state_flow.print_state()
            print(distance)


# instance
state_flow = State_Flow_Setup()

count = 1 #繰り返し数
height = 0.0 #現在の台座の高さ(cm)
distance = 0.0 #現在のロボットの位置（cm）

state = STATEMACHINE()
state.start()