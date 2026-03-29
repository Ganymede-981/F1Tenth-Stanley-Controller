import gym
import numpy as np
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env
import f110_gym

# --- Create the F1TENTH environment ---
env = f110_gym.F110Env(map='maps/Budapest', num_agents=1)
check_env(env, warn=True)  # Optional sanity check

# --- Wrap env in a Gym-style interface (needed for SB3) ---
class F110Wrapper(gym.Env):
    def __init__(self):
        super(F110Wrapper, self).__init__()
        self.env = f110_gym.F110Env(map='maps/Budapest', num_agents=1)
        obs, _ = self.env.reset()
        self.observation_space = gym.spaces.Box(low=-np.inf, high=np.inf, shape=obs.shape, dtype=np.float32)
        self.action_space = gym.spaces.Box(low=-1.0, high=1.0, shape=(2,), dtype=np.float32)  # [steer, speed]

    def reset(self):
        obs, _ = self.env.reset()
        return obs

    def step(self, action):
        obs, reward, done, info = self.env.step(action)
        return obs, reward, done, info

    def render(self, mode='human'):
        self.env.render(mode=mode)

# --- Create RL environment ---
env = F110Wrapper()

# --- Create PPO model ---
model = PPO(
    "MlpPolicy",
    env,
    verbose=1,
    tensorboard_log="./ppo_f110_log/",
    learning_rate=3e-4,
    n_steps=1024,
    batch_size=64,
)

# --- Train ---
model.learn(total_timesteps=10000)  # start small for testing

# --- Save model ---
model.save("ppo_f110_test")
print("✅ Training complete and model saved!")
