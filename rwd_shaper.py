import gymnasium as gym
import five_bar
from stable_baselines3 import A2C,PPO,SAC, TD3, DDPG
import os
import sys
from stable_baselines3.common.env_checker import check_env
import optuna
import numpy as np


LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
TRAIN_DIR=os.path.join(os.path.dirname(__file__), 'trained_agents')

algo_dict = {
        "ddpg": DDPG,
        "ppo": PPO,
        "a2c": A2C,
        "sac": SAC,
        "td3": TD3
    }

def param_str(params):
    """Generate a string representation of parameters for file naming."""
    return f"_enA-{params['enA']:.2f}_diA-{params['diA']:.2f}_enB-{params['enB']:.2f}_diB-{params['diB']:.2f}"

def train(env,algo_name,params,timesteps):

    path=os.path.join(TRAIN_DIR,algo_name+param_str(params),"agent")
    logpath=os.path.join(LOG_DIR,algo_name+param_str(params))

    if algo_name.lower() not in algo_dict:
        raise ValueError(f"Algorithm {algo_name} not recognized.")

    model = algo_dict[algo_name.lower()]('MlpPolicy', env, verbose=1, device="cuda", tensorboard_log=logpath)
    model.learn(total_timesteps=timesteps, progress_bar=True)
    model.save(path)

    del model
    
def model_load(env,algo_name,params):
    
    path=os.path.join(TRAIN_DIR,algo_name+param_str(params),"agent")
    
    if algo_name.lower() not in algo_dict:
        raise ValueError(f"Algorithm {algo_name} not recognized.")

    return algo_dict[algo_name.lower()].load(path, env=env)

def objective(trial):
    """Objective function for Optuna hyperparameter tuning."""
    algo_name="sac"   
    # Sample parameters
    params = {
        "enA": trial.suggest_float("enA", 0.1, 10.0),
        "diA": trial.suggest_float("diA", 0.1, 10.0),
        "enB": trial.suggest_float("enB", 0.1, 10.0),
        "diB": trial.suggest_float("diB", 0.1, 10.0),
    }

    # Create environment with sampled parameters
    env = gym.make("five_bar-v0", render_mode=None, camera_name="free",
                   reward_dist_weight_A=params["diA"],
                   reward_dist_weight_B=params["diB"],
                   reward_control_weight_A=params["enA"],
                   reward_control_weight_B=params["enB"])
    
    model = train(env, algo_name, params, timesteps=2_000_000)

    energies, distances = [], []
    current_energy, current_distance = [], []
    
    episodes = 0
    while episodes < 1000:
        action, _ = model.predict(obs, deterministic=True)
        obs, reward, done, info = vec_env.step(action)
        #vec_env.render("human")

        current_energy.append(info[0]["energy"])
        current_distance.append(info[0]["distance"])

        if done:
            episodes += 1
            energies.append(current_energy[:])
            distances.append(current_distance[:])
            current_energy.clear()
            current_distance.clear()
    max_length = max(map(len, energies))
    pad = lambda series: np.array([np.pad(s, (0, max_length - len(s)), constant_values=np.nan) for s in series])
    energy_arr, distance_arr = pad(energies), pad(distances)

    # Compute averages
    avg_energy, avg_distance = np.nanmean(energy_arr, axis=0), np.nanmean(distance_arr, axis=0)
    total_energy=np.nansum(avg_energy)
    precision=avg_distance[-1]

    return total_energy+precision*100

if __name__ == "__main__":
    study = optuna.create_study(direction="minimize")  # Maximize the mean reward
    study.optimize(objective, n_trials=20)  # Run 20 trials

    print("Best Parameters:", study.best_params)
    
    
    

