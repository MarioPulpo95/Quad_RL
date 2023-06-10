import rospkg
from stable_baselines3.common.callbacks import CheckpointCallback,EvalCallback,StopTrainingOnNoModelImprovement, BaseCallback
from stable_baselines3.common.logger import HParam
from stable_baselines3 import DDPG, SAC, TD3, PPO,A2C
from stable_baselines3.common.env_checker import check_env
from stable_baselines3.common.evaluation import evaluate_policy
import gym
import torch as th
from stable_baselines3.common.monitor import Monitor
import rospy
import numpy as np
from stable_baselines3.common.noise import OrnsteinUhlenbeckActionNoise
from stable_baselines3.common.noise import NormalActionNoise


class HParamCallback(BaseCallback):

    def _on_training_start(self) -> None:
        hparam_dict = {
            "algorithm": self.model.__class__.__name__,
            "learning rate": self.model.learning_rate,
            "gamma": self.model.gamma,
            "batch_size": self.model.batch_size
        }
        metric_dict = {
            "rollout/ep_len_mean": 0,
            "train/value_loss": 0.0,
            "rollout/ep_rew_mean": 0.0
        }
        self.logger.record(
            "hparams",
            HParam(hparam_dict, metric_dict),
            exclude=("stdout", "log", "json", "csv"),
        )

    def _on_step(self) -> bool:
        return True   
    
def start_train():
    seed = 42
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rotors_gazebo')
    log_path = pkg_path + '/Training/Logs/NewTrains/'

    folder = 'SAC/'
    algo = 'SAC5_NOTILT'
    save_path = pkg_path + '/Training/Saved Models/' + folder + algo

    env = gym.make('Quad-v0') # Quad-v1
    check_env(env)
    env = Monitor(env, log_path, override_existing=False)
    env.seed(42)

    stop_train_callback = StopTrainingOnNoModelImprovement(max_no_improvement_evals=3, min_evals=10, verbose=1)
    eval_callback = EvalCallback(eval_env = env, best_model_save_path=save_path, eval_freq=3000 ,deterministic=True, render=False, callback_after_eval=stop_train_callback, verbose=1)

    checkpoint_callback = CheckpointCallback(
        save_freq=10000,
        save_path=save_path,
        name_prefix= algo,
        save_replay_buffer=False,
        save_vecnormalize=False,
    )

    #action_noise = OrnsteinUhlenbeckActionNoise(mean=np.zeros(3), sigma=0.1, theta=0.15)
    action_noise = NormalActionNoise(mean=np.zeros(3), sigma=0.1)

    learning_rate = 0.0003
    batch_size = 128
    buffer_size = 50000 
    total_timesteps = 200000
    gamma = 0.99
    n_steps = 200
    n_epochs = 10
    action_noise = action_noise
    ent_coef = 0.1

    
    policy_kwargs = dict(activation_fn = th.nn.Tanh, net_arch=[128,128])  
    model =    SAC("MlpPolicy",
                    env=env, 
                    verbose = 0, 
                    tensorboard_log = log_path,
                    policy_kwargs = policy_kwargs,
                    batch_size = batch_size, 
                    learning_rate = learning_rate,
                    gamma=gamma,
                    #ent_coef=ent_coef,
                    #n_steps=n_steps, # PPO
                    #n_epochs=n_epochs, # PPO
                    buffer_size=buffer_size,
                    #action_noise=action_noise,
                    learning_starts=1000,
                    seed=seed
                    )

        
    model.learn(total_timesteps=total_timesteps, progress_bar=True, callback=[checkpoint_callback, HParamCallback()], reset_num_timesteps=True, tb_log_name=algo)
    '''mean_reward, std_reward = evaluate_policy(model=model, env=env, n_eval_episodes=10)'''
    #model = SAC.load(save_path + '/SAC4_NOTILT_160000_steps', env)
    #model.learn(total_timesteps=total_timesteps, progress_bar=True, callback=[checkpoint_callback, HParamCallback()], reset_num_timesteps=False, tb_log_name=algo)

def load():
    seed = 42
    rospack = rospkg.RosPack()
    pkg_path = rospack.get_path('rotors_gazebo')
    log_path = pkg_path + '/Training/Logs/NewTests/'

    folder = 'TD3/'
    algo = 'TD34_WITHTILT'
    save_path = pkg_path + '/Training/Saved Models/' + folder + algo

    env = gym.make('Quad-v2') 
    check_env(env)
    env = Monitor(env, log_path, override_existing=False)
    env.seed(seed=seed)

    model = TD3.load(save_path + '/TD34_WITHTILT_170000_steps', env)
    #model.learn(total_timesteps=100000, progress_bar=True, callback=[checkpoint_callback, HParamCallback()], reset_num_timesteps=False, tb_log_name=algo)

    mean_reward, std_reward = evaluate_policy(model=model, env=env, n_eval_episodes=1, return_episode_rewards=True)
    #rospy.loginfo('mean_reward:{} std_reward:{}'.format(mean_reward,std_reward))

        