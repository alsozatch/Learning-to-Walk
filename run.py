import gym
import testgym
from baselines import deepq

def callback(lcl, glb):
    # stop training if reward exceeds 199
    is_solved = lcl['t'] > 100 and sum(lcl['episode_rewards'][-101:-1]) / 100 >= 199
    return is_solved

def main():
    
    #env = gym.make("mygym-v0")
    #env._reset() #why do we need to call reset manually when supposedly init should call reset, and init is called by gym.make?
    # one answer may be that this guide is old ( hence the use of _step, _render, _reset instead of without underscores) and as such the init calling reset functionality has been removed since then
    #for _ in range(1000):
    #    env._step(6)
    
    #use np.random for noise
    
    env = gym.make("testgym-v0")
    model = deepq.models.mlp([16, 16])
    act = deepq.learn(env, q_func=model, lr=1e-3, max_timesteps=200000, exploration_fraction=0.1, exploration_final_eps=0.02, print_freq=10, callback=callback)
    
    act.save("walkbot.pkl")
    
if __name__ == '__main__':
    main()
