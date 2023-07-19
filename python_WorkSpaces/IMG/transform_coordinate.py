import numpy as np

class Transform_Coordinate:
        '''
        座標変換
        input:
                画像座標系における送風位置: u, v(pixel)
                カメラからみた送風位置の奥行（トマトの花の奥行の平均値）: Z(cm)
                カメラの内部パラメータ  焦点距離: fx, fy、中心座標: cx, cy
        output:
                絶対座標系からみたトマト株の位置 X, Y, Z (cm)
                送風機座標系からみた送風位置の座標 X, Y, Z(cm)
        
        [重要] 以下、変更するように
        camera_p_pos = np.array([-100.0, height, 50.0]) # 全体カメラ座標系　yは変化する。x, zは固定
        machine_pos = np.array([50.0, -90.0, height]) # 送風機座標系 zは変化する。
        '''

        def __init__(self, robot_pos, camera_w_pos, arm_pos, camera_w_rotation_matlix, camera_p_rotation_matlix, machine_rotation_matlix, R_w, R_p):
                self.WIDTH = 640
                self.HEIGHT = 480
                self.robot_pos = robot_pos # ロボット座標系（基準となる座標系）固定
                self.camera_w_pos = camera_w_pos # 全体カメラ座標系　固定
                self.arm_pos = arm_pos
                self.camera_w_rotation_matlix = camera_w_rotation_matlix #全体カメラの回転行列
                self.camera_p_rotation_matlix = camera_p_rotation_matlix #手先カメラの回転行列
                self.machine_rotation_matlix = machine_rotation_matlix #手先カメラの回転行列
                self.camera_w_translation_vector = self.camera_w_pos - self.robot_pos # 全体カメラ-ロボット座標系の並進行列
                self.fx_w, self.fy_w, self.cx_w, self.cy_w = R_w
                self.fx_p, self.fy_p, self.cx_p, self.cy_p = R_p

        def transformation_w(self, arr):
                target_coordinates = []
                for i in range(len(arr)):
                        u, v, Z = arr[i]
                        if Z == 0:
                                continue
                        else:      
                                # img→cam
                                camera_coordinates=self.image_to_camera(u, v, Z, self.fx_w, self.fy_w, self.cx_w, self.cy_w)
                                # img→robot
                                robot_coordinates=self.camera_to_robot(camera_coordinates, self.camera_w_rotation_matlix, self.camera_w_translation_vector)
                                target_coordinates.append(robot_coordinates)
                return np.array(target_coordinates)
        
        def transformation_p(self, arr, camera_p_pos, machine_pos):
                target_coordinates = []
                for i in range(len(arr)):
                        u, v, Z = arr[i]
                        if Z == 0:
                                continue
                        else:
                                camera_p_translation_vector = camera_p_pos - self.robot_pos # 手先カメラ-ロボット座標系の並進行列
                                machine_translation_vector = self.robot_pos - machine_pos
                                # img→cam
                                camera_coordinates=self.image_to_camera(u, v, Z, self.fx_p, self.fy_p, self.cx_p, self.cy_p)
                                # img→robot
                                robot_coordinates=self.camera_to_robot(camera_coordinates, self.camera_p_rotation_matlix, camera_p_translation_vector)
                                # img→machine
                                machine_coordinates=self.robot_to_target(robot_coordinates, self.machine_rotation_matlix, machine_translation_vector)
                                target_coordinates.append(machine_coordinates)           
                return np.array(target_coordinates)
        
        def transformation_a(self, arr):
                arm_translation_vector = self.robot_pos - self.arm_pos # ロボット -アーム座標系の並進行列
                target_coordinates = []
                for i in range(len(arr)):
                        u, v, Z = arr[i]
                        if Z == 0:
                                continue
                        else:
                                # img→cam
                                camera_coordinates=self.image_to_camera(u, v, Z, self.fx_w, self.fy_w, self.cx_w, self.cy_w)
                                # img→robot
                                robot_coordinates=self.camera_to_robot(camera_coordinates, self.camera_w_rotation_matlix, self.camera_w_translation_vector)
                                # img→machine
                                machine_coordinates=self.robot_to_target(robot_coordinates, self.machine_rotation_matlix, arm_translation_vector)
                                target_coordinates.append(machine_coordinates)           
                return np.array(target_coordinates)
        

        def image_to_camera(self, u, v, Z, fx, fy, cx, cy): #ok
                X_c = (u - cx) * Z / fx 
                Y_c = ((self.HEIGHT - v) - cy) * Z / fy
                Z_c = Z
                camera_coordinates = np.array([X_c, Y_c, Z_c])
                return camera_coordinates
        
        def camera_to_robot(self, coordinates, rotation_matrix, translation_vector):
                robot_coordinates = rotation_matrix @ coordinates + translation_vector
                keep_side, keep_height, keep_depth = robot_coordinates
                return np.array([keep_depth, keep_side, keep_height])

        def robot_to_target(self, coordinates, rotation_matrix, translation_vector):
                target_coordinates = rotation_matrix @ coordinates + translation_vector
                return target_coordinates

'''param'''
# height = 130
# robot_pos = np.array([0.0, 0.0, 0.0]) # ロボット座標系（基準となる座標系）固定
# camera_w_pos = np.array([-6.5, 106.0, 0.0]) # 全体カメラ座標系　固定
# camera_p_pos = np.array([-100.0, height, 50.0]) # 全体カメラ座標系　yは変化する。x, zは固定
# arm_pos = np.array([63.0, -40.0, 78.0])
# machine_pos = np.array([50.0, -90.0, height]) # 送風機座標系 zは変化する。
# camera_w_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #全体カメラの回転行列
# camera_p_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
# machine_rotation_matlix = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]]) #手先カメラの回転行列
# R_w = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])
# R_p = np.array([618.6954956054688, 618.8485717773438, 320.0, 240.0])

'''debug'''
# node = Transform_Coordinate(robot_pos, camera_w_pos, arm_pos, camera_w_rotation_matlix, camera_p_rotation_matlix, machine_rotation_matlix, R_w, R_p)
# arr = np.array([[230,  180, 70]])
# w = node.transformation_w(arr)
# print(w)
# p = node.transformation_p(arr, camera_p_pos, machine_pos)
# print(p)

# a = node.transformation_a(arr)
# print(a)