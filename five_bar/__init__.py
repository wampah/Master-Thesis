from gymnasium.envs.registration import register
register(id='five_bar-v0',
         entry_point='five_bar.envs.fivebar_reacher:FiveBar_Reacher',
         max_episode_steps=30,
        reward_threshold=1000000,)