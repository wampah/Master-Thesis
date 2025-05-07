__credits__ = ["Juan Pablo Reyes"]


import numpy as np
import pandas as pd
import os
from typing import Dict, Union,Optional
from gymnasium import utils, error, spaces
from gymnasium.envs.mujoco import MujocoEnv

DEFAULT_CAMERA_CONFIG = {
    "distance": 1.5,
    "trackbodyid": 0
    }

FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "assets","5_bar.xml")
DATA_PATH = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(__file__))), "dynamic analysis","data","initial_pts.parquet")

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
        reward_dist_weight_A: float = 1,
        reward_control_weight_A: float = 1,
        reward_dist_weight_B: float = 1,
        reward_control_weight_B: float = 1,
        reset_robot_pos_every_episode: bool =False,
        specify_data_configuration=None,
        specify_data_trim_x=None,
        specify_data_trim_y=None,
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
            reset_robot_pos_every_episode,
            specify_data_configuration=None,
            specify_data_trim_x=None,
            specify_data_trim_y=None,
            **kwargs,
        )        
        
        self._reward_dist_weight_A = reward_dist_weight_A
        self._reward_control_weight_A = reward_control_weight_A
        self._reward_dist_weight_B = reward_dist_weight_B
        self._reward_control_weight_B = reward_control_weight_B
        
        data=pd.read_parquet(DATA_PATH)

        if specify_data_configuration == '(-,+)':
            self.initial_pts=data[data["case"]=='(-,+)']
            
        elif specify_data_configuration == '(+,+)':
            self.initial_pts=data[data["case"]=='(+,+)']
            
        elif specify_data_configuration == '(+,-)':
            self.initial_pts=data[data["case"]=='(+,-)']
            
        elif specify_data_configuration == '(-,-)':
            self.initial_pts=data[data["case"]=='(-,-)']
            
        else:
            self.initial_pts=data
            
        if specify_data_trim_x=="positive":
            self.initial_pts=self.initial_pts[self.initial_pts["eff_x"]>0]
        elif specify_data_trim_x=="negative":
            self.initial_pts=self.initial_pts[self.initial_pts["eff_x"]<0]
        else:
            pass
        if specify_data_trim_y=="positive":
            self.initial_pts=self.initial_pts[self.initial_pts["eff_y"]>0]
        elif specify_data_trim_y=="negative":
            self.initial_pts=self.initial_pts[self.initial_pts["eff_y"]<0]
        else:
            pass
        
        self.reset_robot_pos_every_episode=reset_robot_pos_every_episode
        
        observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(10,), dtype=np.float64)

        MujocoEnv.__init__(
            self, 
            xml_file, 
            frame_skip,
            observation_space=observation_space,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
            )
        
        self.model_initial_qpos=self.data.qpos.copy()
        self.model_initial_qvel=self.data.qvel.copy()
        
        # Rescale actions
        self.original_low = np.array([-1.6, -1.6])  # Torques - This has to agree with the xml model!
        self.original_high = np.array([1.6, 1.6])

        self.action_space = spaces.Box(low=-1.0, high=1.0, shape=self.original_low.shape, dtype=np.float32)
        
        self.goal=[None]


    def step(self, action):
        real_action = self.original_low + (action + 1.0) * 0.5 * (self.original_high - self.original_low)

        self.do_simulation(real_action, self.frame_skip)
        
        observation=self._get_obs()
        
        distance=np.linalg.norm((self.get_body_com("end_effector")-self.get_body_com("target"))[:2])
        energy=self.model.opt.timestep * (abs(self.data.qvel[0] * real_action[0]) + abs(self.data.qvel[2] * real_action[1]))
        
        reward_dist = self._reward_dist_weight_A*np.exp(-self._reward_dist_weight_B*distance)
        reward_ctrl = self._reward_control_weight_A*np.exp(-self._reward_control_weight_B*energy)
        
        reward = reward_dist + reward_ctrl

        reward_info = {
            "distance": distance,
            "energy": energy,
        }
        
        if (self.render_mode=="human"):
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
        
        self.goal=self.initial_pts.iloc[random_index2]

        if self.reset_robot_pos_every_episode==True:
            qpos = np.array([j1,j3,j2,j4,self.goal['eff_x'],self.goal["eff_y"]])
        elif self.reset_robot_pos_every_episode==False:
            qpos = np.array([self.data.qpos[0],self.data.qpos[1],self.data.qpos[2],self.data.qpos[3],self.goal['eff_x'],self.goal["eff_y"]])
        else:
            qpos = np.array([self.model_initial_qpos[0],self.model_initial_qpos[1],self.model_initial_qpos[2],self.model_initial_qpos[3],self.goal['eff_x'],self.goal["eff_y"]])
            
        
        qvel=np.zeros(self.model.nv)

        self.set_state(qpos, qvel)
        return self._get_obs()
    
    def reset(
        self,
        *,
        seed: Optional[int] = None,
        options: Optional[dict] = None,
    ):
        
        ob = self.reset_model()
        info = self._get_reset_info()

        if self.render_mode == "human":
            self.render()
        return ob, info
    
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
        
    def get_simulation_state(self):
        return {
            "time": self.data.time,
            "qpos": self.data.qpos.copy(),
            "qvel": self.data.qvel.copy(),
            "ctrl": self.data.ctrl.copy(),
            "goal": self.goal.copy()
        }
        