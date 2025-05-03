import gymnasium as gym
import five_bar
from stable_baselines3 import A2C,PPO,SAC, TD3, DDPG
from stable_baselines3.common.env_checker import check_env

import os
import sys
import matplotlib.pyplot as plt
import numpy as np
def model_load(env,model_name):
    
    algo_name=model_name.split("_")[0]
        
    path=os.path.join(os.path.dirname(__file__), 'trained_agents',model_name,"agent.zip")
    
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
        model_name = sys.argv[1]
        
        env = gym.make("five_bar-v0", render_mode="human", camera_name="free")
        # print("Checking Env...")
        # check_env(env)
        # print("Env check finished")
        model=model_load(env,model_name)
        
        vec_env = model.get_env()
        
        obs = vec_env.reset()
        energies, distances = [], []
        current_energy, current_distance = [], []
        
        angle1,angle2= [], []

        episodes = 0
        while episodes < 1:
            action, _ = model.predict(obs, deterministic=True)
            obs, reward, done, info = vec_env.step(action)

            current_energy.append(info[0]["energy"])
            current_distance.append(info[0]["distance"])

            angle1.append(info[0]["qpos"][0])
            angle2.append(info[0]["qpos"][2])
            if done:
                episodes += 1
                energies.append(current_energy[:])
                distances.append(current_distance[:])
                current_energy.clear()
                current_distance.clear()

        # Pad series for averaging
        max_length = max(map(len, energies))
        pad = lambda series: np.array([np.pad(s, (0, max_length - len(s)), constant_values=np.nan) for s in series])
        energy_arr, distance_arr = pad(energies), pad(distances)

        # Compute averages
        avg_energy, avg_distance = np.nanmean(energy_arr, axis=0), np.nanmean(distance_arr, axis=0)
        print(f"Total Energy: {np.nansum(avg_energy):.2f}, Last Distance: {avg_distance[-1]:.2f}")

        # Plot all subplots
        fig, axes = plt.subplots(4, 1, figsize=(10, 12))
        titles = ["Energy Series", "Distance Series", "Average Energy", "Average Distance"]
        data = [energies, distances, [avg_energy], [avg_distance]]
        colors = [None, None, ["red"], ["blue"]]

        for ax, title, series, color in zip(axes, titles, data, colors):
            for i, s in enumerate(series):
                if color:  # Apply color only when it's specified
                    ax.plot(s, color=color[0], label="Average")
                    ax.legend()
                else:
                    ax.plot(s)
            ax.set(title=title, xlabel="Time Step")
            
            ax.grid(True)

        plt.tight_layout()
        plt.show()