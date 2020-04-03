from gym.envs.registration import register

register(
    id='testgym-v0',
    entry_point='testgym.envs:TestGymEnv',
)
