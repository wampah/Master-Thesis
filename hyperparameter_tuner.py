import gymnasium as gym
import five_bar
from stable_baselines3 import A2C,PPO,SAC, TD3, DDPG
import os
import sys
from stable_baselines3.common.env_checker import check_env
import optuna



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

def objective(trial,algo_name):
    """Objective function for Optuna hyperparameter tuning."""

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
    


    
if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python train.py <algorithm>")
        print("Available algorithms: 'ddpg', 'ppo', 'a2c', 'sac' or td3.")
    else:
        algo_name = sys.argv[1]
        
        params={"enA":1,"diA":1,"enB":1,"diB":1}

        env = gym.make("five_bar-v0", render_mode=None, camera_name="free",
                       reward_dist_weight_A= params["diA"],
                       reward_dist_weight_B= params["diB"],
                       reward_control_weight_A = params["enA"],
                       reward_control_weight_B= params["enB"])
        
        env_render = gym.make("five_bar-v0", render_mode="human", camera_name="free",
                       reward_dist_weight_A= params["diA"],
                       reward_dist_weight_B= params["diB"],
                       reward_control_weight_A = params["enA"],
                       reward_control_weight_B= params["enB"])

        print("-"*50,"Checking Env","-"*50)
        check_env(env)
        print("-"*50,"Done Checking Env","-"*50)
        train(env,algo_name,params,200)
        
        model=model_load(env_render,algo_name,params)
        
        vec_env = model.get_env()
        
        obs= vec_env.reset()
        for i in range(10000):
            action, _state = model.predict(obs, deterministic=True)
            obs, reward, done, info = vec_env.step(action)

    
    
    

