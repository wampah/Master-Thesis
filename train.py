import gymnasium as gym
import five_bar
from stable_baselines3 import DDPG

env = gym.make("five_bar-v0", render_mode="rgb_array", camera_name="free")


# model = DDPG("MlpPolicy", env, verbose=1)
# model.learn(total_timesteps=500_000)

# model.save("DDPG_5bar")
# del model
model = DDPG.load("DDPG_5bar", env=env)

vec_env = model.get_env()
obs = vec_env.reset()
for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render("human")
    # VecEnv resets automatically
    # if done:
    #   obs = vec_env.reset()