import os
import math
import time
import numpy as np
import sim
import simConst
from dotenv import load_dotenv

from coppeliasim_client import Coppeliasim_client

load_dotenv()

class HT_matrix:
    def __init__(self):
        self.dh_d     = self.parse_dh_param(os.getenv("DH_D"))
        self.dh_a     = self.parse_dh_param(os.getenv("DH_A"))
        self.dh_alpha = self.parse_dh_param(os.getenv("DH_ALPHA"))
        
        self.T_base_to_world = np.array([
            [ 0,  1,  0,  0],  # Base의 Y가 World의 X가 됨
            [-1,  0,  0,  0],  # Base의 X가 World의 -Y가 됨
            [ 0,  0,  1,  0],  # Z는 그대로
            [ 0,  0,  0,  1]
        ])

    def parse_dh_param(self, env_str):
        if not env_str: return [0]*6
        safe_env_str = env_str.replace('pi', str(math.pi))
        try:
            return [float(eval(item)) for item in safe_env_str.split(',')]
        except Exception as e:
            print(f"DH Parameter Parsing Error: {e}")
            return [0]*6

    def get_dh_Transform(self, theta, d, a, alpha):
        return np.array([
            [math.cos(theta), -math.sin(theta)*math.cos(alpha),  math.sin(theta)*math.sin(alpha), a*math.cos(theta)],
            [math.sin(theta),  math.cos(theta)*math.cos(alpha), -math.cos(theta)*math.sin(alpha), a*math.sin(theta)],
            [0,                math.sin(alpha),                  math.cos(alpha),                 d],
            [0,                0,                                0,                               1]
        ])

    def ForwardKinematics(self, joint_angles):
        # 각도 보정
        theta = [
            joint_angles[0],
            joint_angles[1] - math.pi/2, 
            joint_angles[2],
            joint_angles[3] - math.pi/2,
            joint_angles[4],
            joint_angles[5]
        ]

        T_final = np.eye(4)
        for i in range(6):
            T_i = self.get_dh_Transform(theta[i], self.dh_d[i], self.dh_a[i], self.dh_alpha[i])
            T_final = np.dot(T_final, T_i)
            
        # World 좌표계 변환 적용
        self.T06 = np.dot(self.T_base_to_world, T_final)
        return self.T06
    
    def get_position(self):
        return self.T06[:3, 3]
