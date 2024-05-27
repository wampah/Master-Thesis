import gymnasium as gym
import five_bar
from stable_baselines3 import A2C,PPO,SAC, TD3, DDPG
import os
import sys

def train(env,algo_name):
    path=os.path.join(os.path.dirname(__file__), 'training_results\\'+algo_name)
    
    if algo_name.lower() == 'ddpg':
        model = DDPG('MlpPolicy', env, verbose=1)
    elif algo_name.lower() == 'ppo':
        model = PPO('MlpPolicy', env, verbose=1)
    elif algo_name.lower() == 'a2c':
        model = A2C('MlpPolicy', env, verbose=1)
    elif algo_name.lower() == 'sac':
        model = SAC('MlpPolicy', env, verbose=1)
    elif algo_name.lower() == 'td3':
        model = TD3('MlpPolicy', env, verbose=1)
    else:
        raise ValueError(f"Algorithm {algo_name} not recognized. Please choose 'ddpg', 'ppo', 'a2c', 'sac' or td3.")
    
    model.learn(total_timesteps=500_000)

    model.save(path)
    
    del model
    
def model_load(env,algo_name):
    
    path=os.path.join(os.path.dirname(__file__), 'training_results\\'+algo_name)
    
    if algo_name.lower() == 'ddpg':
        model = DDPG.load(path, env=env)
    elif algo_name.lower() == 'ppo':
        model = PPO.load(path, env=env)
    elif algo_name.lower() == 'a2c':
        model = A2C.load(path, env=env)
    elif algo_name.lower() == 'sac':
        model = SAC.load(path, env=env)
    elif algo_name.lower() == 'td3':
        model = TD3.load(path, env=env)
    else:
        raise ValueError(f"Algorithm {algo_name} not recognized. Please choose 'ddpg', 'ppo', 'a2c', 'sac' or td3.")
    
    return model
    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python train.py <algorithm>")
        print("Available algorithms: 'ddpg', 'ppo', 'a2c', 'sac' or td3.")
    else:
        algo_name = sys.argv[1]
        
        env = gym.make("five_bar-v0", render_mode="rgb_array", camera_name="free")
        
        train(env,algo_name)
        
        model=model_load(env,algo_name)
        
        vec_env = model.get_env()
        
        obs = vec_env.reset()
        for i in range(10000):
            action, _state = model.predict(obs, deterministic=True)
            obs, reward, done, info = vec_env.step(action)
            vec_env.render("human")
            # VecEnv resets automatically
            # if done:
            #   obs = vec_env.reset()
        
    
    
    

