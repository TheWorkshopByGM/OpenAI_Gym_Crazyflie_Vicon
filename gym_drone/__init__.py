from gym.envs.registration import register

register(
    id='gym_drone-v0',
    entry_point='gym_drone.envs:CustomEnv',
    max_episode_steps=10000,
)