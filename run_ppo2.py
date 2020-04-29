import gym
import testgym
from baselines.common.vec_env.dummy_vec_env import DummyVecEnv
from baselines.ppo2.ppo2 import learn

def main():
    def env_fn():
        env = gym.make("testgym-v0")
        return env
        
    model = learn(network='mlp', env=DummyVecEnv([env_fn]), total_timesteps=100000)
    
    model.save("walkbot.pkl")
    
if __name__ == '__main__':
    main()
