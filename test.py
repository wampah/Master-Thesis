import gymnasium as gym

from stable_baselines3 import DDPG

env = gym.make("Reacher-v4", render_mode="rgb_array")

model = DDPG("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100_000)

model.save("DDPG_reacher")
del model
model = DDPG.load("DDPG_reacher", env=env)

vec_env = model.get_env()
obs = vec_env.reset()
for i in range(1000):
    action, _state = model.predict(obs, deterministic=True)
    obs, reward, done, info = vec_env.step(action)
    vec_env.render("human")
    # VecEnv resets automatically
    # if done:
    #   obs = vec_env.reset()