from gymnasium.envs.registration import register
register(id='five_bar-v0',
         entry_point='five_bar.envs.fivebar_reacher:FiveBar_Reacher',)