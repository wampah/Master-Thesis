import numpy as np
import pandas as pd
import os
from gymnasium import utils, error, spaces
from gymnasium.envs.mujoco import MujocoEnv

DEFAULT_CAMERA_CONFIG = {
    "distance": 1.5,
    "trackbodyid": 0
    }

class FiveBar_Reacher(MujocoEnv, utils.EzPickle):
    metadata = {
        "render_modes": [
            "human",
            "rgb_array",
            "depth_array",
        ],
        "render_fps": 50,
    }
    def __init__(self,**kwargs):
        utils.EzPickle.__init__(self,**kwargs)
        FILE_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "assets\\5_barras.xml")
        DATA_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "dynamic analysis\\initial_pts.parquet")
        
        df = pd.read_parquet(DATA_PATH)
        self.initial_pts=df.to_numpy()

        observation_space = spaces.Box(low=-np.inf, high=np.inf, shape=(11,), dtype=np.float64)
        frame_skip = 2
        MujocoEnv.__init__(
            self, 
            FILE_PATH, 
            frame_skip,
            observation_space=observation_space,
            default_camera_config=DEFAULT_CAMERA_CONFIG,
            **kwargs,
            )

    def step(self, action):
        
        vector=self.get_body_com("end_effector")-self.get_body_com("target")
        rwd_distance=-np.linalg.norm(vector)
        rwd_control=-np.square(action).sum()
        
        coef_dist=1
        coef_action=0.1
        
        rwd=coef_dist*rwd_distance+coef_action*rwd_control
        
        self.do_simulation(action, self.frame_skip)
        if self.render_mode=="human":
            self.render()
        
        observation=self._get_obs()
        
        return (
            observation,
            rwd,
            False,
            False,
            dict(rwd_distance=rwd_distance,rwd_control=rwd_control),
            )
    def reset_model(self):
        
        random_index1 = np.random.choice(self.initial_pts.shape[0])
        random_index2 = np.random.choice(self.initial_pts.shape[0])

        random_qpos = self.initial_pts[random_index1, :]
        
        q_p1=random_qpos[0]
        q_d1=random_qpos[1]
        q_d2=random_qpos[2]
        q_p2=random_qpos[3]
        
        j1=q_p1
        j2=q_p2
        j3=q_d1-q_p1
        j4=q_d2-q_p2
        
        random_effs = self.initial_pts[random_index2, :]

        self.goal=np.array([random_effs[4],random_effs[5]])
        
        print(self.goal)
        
        qpos = np.array([j1,j3,j2,j4,self.goal[0],self.goal[1]])

        qvel=np.zeros(self.model.nv)

        self.set_state(qpos, qvel)
        return self._get_obs()
    
    def _get_obs(self):
      theta=[self.data.qpos[0],self.data.qpos[2]]

      omega=[self.data.qvel[0],self.data.qvel[2]]
      
      target_pos=[self.data.qpos[4],self.data.qpos[5]]


      return np.concatenate(
          [
              np.cos(theta),
              np.sin(theta),
              target_pos,
              omega,
              self.get_body_com("end_effector")-self.get_body_com("target"),
          ]
      )