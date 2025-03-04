__credits__ = ["Juan Pablo Reyes"]


import numpy as np
import pandas as pd
import os
from typing import Dict, Union
from gymnasium import utils, error, spaces
from gymnasium.envs.mujoco import MujocoEnv

DEFAULT_CAMERA_CONFIG = {
    "distance": 1.5,
    "trackbodyid": 0
    }

FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets\\5_bar.xml")
DATA_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "dynamic analysis\\data\\initial_pts.parquet")

class FiveBar_Reacher(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 50,
    }
    def __init__(
        self,
        xml_file: str = FILE_PATH,
        frame_skip: int = 2,
        default_camera_config: Dict[str, Union[float, int]] = DEFAULT_CAMERA_CONFIG,
        reward_dist_weight_A: float = 100,
        reward_control_weight_A: float = 1,
        reward_dist_weight_B: float = 1,
        reward_control_weight_B: float = 1,
        **kwargs,
    ):
        utils.EzPickle.__init__(
            self,
            xml_file,
            frame_skip,
            default_camera_config,
            reward_dist_weight_A,
            reward_control_weight_A,
            reward_dist_weight_B,
            reward_control_weight_B,
            **kwargs,
        )        
        
        self._reward_dist_weight_A = reward_dist_weight_A
        self._reward_control_weight_A = reward_control_weight_A
        self._reward_dist_weight_B = reward_dist_weight_B
        self._reward_control_weight_B = reward_control_weight_B
        
        self.initial_pts=pd.read_parquet(DATA_PATH)
        
        observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64)

        MujocoEnv.__init__(
            self, 
            xml_file, 
            frame_skip,
            observation_space=observation_space,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
            )


    def step(self, action):
        
        self.do_simulation(action, self.frame_skip)
        
        observation=self._get_obs()
        
        distance=np.linalg.norm((self.get_body_com("end_effector")-self.get_body_com("target"))[:2])
        energy=self.model.opt.timestep * (abs(self.data.qvel[0] * action[0]) + abs(self.data.qvel[2] * action[1]))
        
        reward_dist = self._reward_dist_weight_A*np.exp(-self._reward_dist_weight_B*distance)
        reward_ctrl = self._reward_control_weight_A*np.exp(-self._reward_control_weight_B*energy)
        
        reward = reward_dist + reward_ctrl

        reward_info = {
            "distance": distance,
            "energy": energy,
        }
        
        if self.render_mode=="human":
            self.render()

        return (
            observation,
            reward,
            False,#Terminated 
            False,#Truncated
            reward_info #Info
            )
    def reset_model(self):
        
        random_index1 = np.random.choice(self.initial_pts.shape[0])
        random_index2 = np.random.choice(self.initial_pts.shape[0])

        random_qpos = self.initial_pts.iloc[random_index1]
                
        q_p1=random_qpos['q3']
        q_d1=random_qpos['q6']
        q_d2=random_qpos['q9']
        q_p2=random_qpos['q12']
        
        j1=q_p1
        j2=q_p2
        j3=q_d1-q_p1
        j4=q_d2-q_p2
        
        random_effs = self.initial_pts.iloc[random_index2]

        self.goal=np.array([random_effs['eff_x'],random_effs["eff_y"]])
                
        qpos = np.array([j1,j3,j2,j4,self.goal[0],self.goal[1]])

        qvel=np.zeros(self.model.nv)

        self.set_state(qpos, qvel)
        return self._get_obs()
    
    def _get_obs(self):
        theta=[self.data.qpos[0],self.data.qpos[2]]# 0 and 2 are effectors

        omega=[self.data.qvel[0],self.data.qvel[2]]

        target_pos=[self.data.qpos[4],self.data.qpos[5]]

        return np.concatenate(
            [
                np.cos(theta),
                np.sin(theta),
                target_pos,
                omega,
                (self.get_body_com("end_effector")-self.get_body_com("target"))[:2]
            ]
        )