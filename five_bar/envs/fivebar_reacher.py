import numpy as np
import os
from gymnasium import utils, error, spaces
from gymnasium.envs.mujoco import MujocoEnv

DEFAULT_CAMERA_CONFIG = {"trackbodyid": 0}

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
        rwd=rwd_distance+rwd_control
        
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
        qpos = (
            self.np_random.uniform(low=-0.1, high=0.1, size=self.model.nq)
            + self.init_qpos
        )
        while True:
            self.goal = self.np_random.uniform(low=-1, high=1, size=2)
            if np.linalg.norm(self.goal) < 0.5:
                break
        qpos[-2:] = self.goal
        qvel = self.init_qvel + self.np_random.uniform(
            low=-0.005, high=0.005, size=self.model.nv
        )
        qvel[-2:] = 0
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