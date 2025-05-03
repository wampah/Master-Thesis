import gymnasium as gym
import five_bar
from stable_baselines3 import A2C,PPO,SAC, TD3, DDPG
import os
import optuna
import numpy as np
import pickle

from stable_baselines3.common.callbacks import BaseCallback
from tqdm import tqdm

LOG_DIR = os.path.join(os.path.dirname(__file__), "logs")
TRAIN_DIR=os.path.join(os.path.dirname(__file__), 'trained_agents')
STUDY_DIR=os.path.join(os.path.dirname(__file__), 'optuna_studies')

folders=[LOG_DIR,TRAIN_DIR,STUDY_DIR]

class ProgressBarCallback(BaseCallback):
    def __init__(self, total_timesteps):
        super().__init__()
        self.total_timesteps = total_timesteps
        self.pbar = None

    def _on_training_start(self):
        self.pbar = tqdm(total=self.total_timesteps)

    def _on_step(self):
        self.pbar.update(1)
        return True

    def _on_training_end(self):
        self.pbar.close()

for folder in folders:
    os.makedirs(folder, exist_ok=True)

STUDY_PATH=os.path.join(STUDY_DIR,"optuna_study.db")
STORAGE_URL = f"sqlite:///{STUDY_PATH}"


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

    model = algo_dict[algo_name.lower()]('MlpPolicy', env, verbose=0, device="cuda", tensorboard_log=logpath)
    callback = ProgressBarCallback(total_timesteps=timesteps)
    model.learn(total_timesteps=timesteps, callback=callback)
    model.save(path)

    return model
    
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
        "enA": trial.suggest_float("enA", 0.01, 10.0),
        "diA": trial.suggest_float("diA", 0.01, 10.0),
        "enB": trial.suggest_float("enB", 0.01, 10.0),
        "diB": trial.suggest_float("diB", 0.01, 10.0),
    }

    # Create environment with sampled parameters
    env = gym.make("five_bar-v0", render_mode=None, 
                   reward_dist_weight_A=params["diA"],
                   reward_dist_weight_B=params["diB"],
                   reward_control_weight_A=params["enA"],
                   reward_control_weight_B=params["enB"])
    
    model = train(env, algo_name, params, timesteps=5_000_000)

    print("-"*50,"Evaluating Agent","-"*50)
    
    energies, distances = [], []
    current_energy, current_distance = [], []
    
    vec_env = model.get_env()
        
    obs = vec_env.reset()

    episodes = 0
    while episodes < 10000:
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
    print("-"*50,"Done Evaluating Agent","-"*50)
 
    return total_energy,precision

if __name__ == "__main__":
    study = optuna.create_study(
        study_name="Reward Shaping Study",
        storage=STORAGE_URL,
        directions=["minimize", "minimize"],  # Multi-objective optimization
        load_if_exists=True,  # Load the study if it already exists
        )  # Maximize the mean reward
    study.optimize(objective, n_trials=30,n_jobs=5)  # Run 20 trials

    # Get Pareto front solutions
    pareto_solutions = study.best_trials
    for trial in pareto_solutions:
        print(trial.values, trial.params)
    
    
    

