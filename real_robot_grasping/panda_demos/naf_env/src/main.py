#!/usr/bin/env python
import rospy
from rl_task_plugins.msg import DesiredErrorDynamicsMsg
from rl_task_plugins.msg import StateMsg

import argparse
import matplotlib.pyplot as plt
from matplotlib.pyplot import cm
import gym
import numpy as np
import torch
import time
import subprocess
import pickle
import matplotlib.colors as colors
import matplotlib.cm as cmx
from torch.autograd import Variable

#import files...
from naf import NAF
from ounoise import OUNoise
from replay_memory import Transition, ReplayMemory
from environment import Env

subdata = []


def callback(data):
    global subdata
    subdata = data.e


rospy.init_node('DRL_traffic', anonymous=True)


def sim_reset_start():
    subprocess.call("~/catkin_workspace/src/panda_demos/panda_table_launch/scripts/sim_reset_episode.sh", shell=True)
    subprocess.call("~/catkin_workspace/src/panda_demos/panda_table_launch/scripts/sim_2drl_tasks.sh", shell=True)


def sim_reset():
    subprocess.call("~/catkin_workspace/src/panda_demos/panda_table_launch/scripts/sim_reset_episode_fast.sh", shell=True)

def sate_Q_plot(agent, ep):
    Qstates = []
    Qvalues = []

    for i in range(45):
        Qvalues.append([])
        Qstates.append([])
        for j in range(45):
            hej = i / 100.0 - 0.36
            hop = j / 100.0 - 0.24
            st = [hej, hop]
            Qsta = torch.Tensor(st).unsqueeze(0)
            Qact = agent.select_action(Qsta)
            agent.model.eval()
            _, Qvalue, _ = agent.model((Variable(Qsta), Variable(Qact)))
            agent.model.train()

            Qstates[i].append(Qsta)

            Qvalues[i].append(Qvalue)

    for k in range(len(Qvalues)):
        Qcolor = []
        for l in range(len(Qvalues[k])):
            Qcolor.append(Qvalues[k][l].data.numpy()[0][0])
        cmap = plt.cm.viridis
        cNorm = colors.Normalize(vmin=np.min(Qvalues), vmax=np.max(Qvalues))
        scalarMap = cmx.ScalarMappable(norm=cNorm, cmap=cmap)
        colorVal = scalarMap.to_rgba(Qcolor)
        sx, sy = torch.cat(Qstates[k]).numpy().T
        plt.scatter(sx, sy, c=colorVal, edgecolors='face')

    figname = 'State_Q_value_{}_{}'.format(ep, '.png')
    plt.savefig(figname)
    plt.close()


def main():
    global subdata
    t_start = time.time()

    parser = argparse.ArgumentParser(description='PyTorch X-job')
    parser.add_argument('--env_name', default="OurEnv-v0",
                        help='name of the environment')
    parser.add_argument('--gamma', type=float, default=0.99, metavar='G',
                        help='discount factor for reward (default: 0.99)')
    parser.add_argument('--tau', type=float, default=0.001,
                        help='discount factor for model (default: 0.001)')
    parser.add_argument('--ou_noise', type=bool, default=True)
    parser.add_argument('--noise_scale', type=float, default=0.4, metavar='G',
                        help='initial noise scale (default: 0.3)')
    parser.add_argument('--final_noise_scale', type=float, default=0.3, metavar='G',
                        help='final noise scale (default: 0.4)')
    parser.add_argument('--exploration_end', type=int, default=33, metavar='N',
                        help='number of episodes with noise (default: 100)')
    parser.add_argument('--seed', type=int, default=4, metavar='N',
                        help='random seed (default: 4)')
    parser.add_argument('--batch_size', type=int, default=512, metavar='N',
                        help='batch size (default: 512)')
    parser.add_argument('--num_steps', type=int, default=300, metavar='N',
                        help='max episode length (default: 1000)')
    parser.add_argument('--num_episodes', type=int, default=50, metavar='N',
                        help='number of episodes (default: 1000)')
    parser.add_argument('--hidden_size', type=int, default=128, metavar='N',
                        help='hidden size (default: 128)')
    parser.add_argument('--replay_size', type=int, default=1000000, metavar='N',
                        help='size of replay buffer (default: 1000000)')
    parser.add_argument('--save_agent', type=bool, default=True,
                        help='save model to file')
    parser.add_argument('--load_agent', type=bool, default=False,
                        help='load model from file')
    parser.add_argument('--train_model', type=bool, default=True,
                        help='Training or run')
    parser.add_argument('--load_exp', type=bool, default=False,
                        help='load saved experience')
    parser.add_argument('--state_plot', type=bool, default=True,
                        help='plot Q values for environment')
    parser.add_argument('--greedy_steps', type=int, default=5, metavar='N',
                        help='amount of times greedy goes (default: 100)')

    args = parser.parse_args()

    #env = gym.make(args.env_name)

    env = Env()

    #env.seed(args.seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)
    # -- initialize agent, Q and Q' --
    agent = NAF(args.gamma, args.tau, args.hidden_size,
                env.observation_space.shape[0], env.action_space)

    # -- declare memory buffer and random process N
    memory = ReplayMemory(args.replay_size)
    memory_g = ReplayMemory(args.replay_size)
    ounoise = OUNoise(env.action_space.shape[0]) if args.ou_noise else None

    # -- load existing model --
    if args.load_agent:
        agent.load_model(args.env_name, args.batch_size, '.pth')
        print("agent: naf_{}_{}_{}, is loaded").format(args.env_name, args.batch_size, '.pth')
    # -- load experience buffer --
    if args.load_exp:
        with open('/home/aass/catkin_workspace/src/panda_demos/exp_replay.pk1', 'rb') as input:
            memory.memory = pickle.load(input)
            memory.position = len(memory)

    #sate_Q_plot(agent, 50)

    rewards = []
    total_numsteps = 0
    greedy_reward = []
    avg_greedy_reward = []
    upper_reward = []
    lower_reward = []
    steps_to_goal = []
    avg_steps_to_goal = []
    state_plot = []


    sim_reset_start()

    pub = rospy.Publisher('/ee_rl/act', DesiredErrorDynamicsMsg, queue_size=10)
    rospy.Subscriber("/ee_rl/state", StateMsg, callback)
    rate = rospy.Rate(9)
    rate.sleep()


    for i_episode in range(args.num_episodes+1):
        # -- reset environment for every episode --
        sim_reset()
        state = torch.Tensor(subdata).unsqueeze(0)

        # -- initialize noise (random process N) --
        if args.ou_noise:
            ounoise.scale = (args.noise_scale - args.final_noise_scale) * max(
                0, args.exploration_end - i_episode / args.exploration_end + args.final_noise_scale)
            ounoise.reset()

        episode_reward = 0


        while True:
            # -- action selection, observation and store transition --
            action = agent.select_action(state, ounoise) if args.train_model else agent.select_action(state)
            a = action.numpy()[0] * 50
            act_pub = [a[0], a[1]]
            pub.publish(act_pub)
            next_state = torch.Tensor(subdata).unsqueeze(0)
            reward, done, _ = env.calc_shaped_reward(next_state)

            total_numsteps += 1
            episode_reward += reward

            action = torch.Tensor(action)
            mask = torch.Tensor([not done])
            reward = torch.Tensor([reward])



            memory.push(state, action, mask, next_state, reward)
            # if done:
            #     for i in range(total_numsteps % args.num_steps):
            #         a = i+1
            #         memory_g.memory.append(memory.memory[-a])
            #         memory_g.position += 1

            state = next_state

            #-- training --
            # if len(memory_g) > args.batch_size / 2 and len(memory) > args.batch_size/2 and args.train_model:
            #     for _ in range(10):
            #         transitions_b = memory.sample(args.batch_size/2)
            #         transitions_g = memory_g.sample(args.batch_size/2)
            #         for i in range(transitions_g):
            #             transitions_b.append(transitions_g[i])
            #         batch = Transition(*zip(*transitions_b))
            #         agent.update_parameters(batch)

            if len(memory) > args.batch_size and args.train_model:
                for _ in range(10):
                    transitions = memory.sample(args.batch_size)
                    batch = Transition(*zip(*transitions))
                    agent.update_parameters(batch)

            else:
                time.sleep(0.1)
            rate.sleep()

            if done or total_numsteps % args.num_steps == 0:
                break

        pub.publish([0, 0])
        rewards.append(episode_reward)

        # -- plot Q value --
        if i_episode % 10 == 0:

            sate_Q_plot(agent, i_episode)
            # -- saves model --
            if args.save_agent:
                agent.save_model(args.env_name, args.batch_size, i_episode, '.pth')
                with open('exp_replay.pk1', 'wb') as output:
                    pickle.dump(memory.memory, output, pickle.HIGHEST_PROTOCOL)
                #with open('exp_replay_g.pk1', 'wb') as output:
                    #pickle.dump(memory_g.memory, output, pickle.HIGHEST_PROTOCOL)

        if args.train_model:
            greedy_episode = max(args.num_episodes/100, 5)
        else:
            greedy_episode = 10
        greedy_range = min(args.greedy_steps, greedy_episode)

        # -- calculates episode without noise --
        if i_episode % greedy_episode == 0 and not i_episode == 0:
            for _ in range(0, greedy_range+1):
                # -- reset environment for every episode --
                sim_reset()
                state_visited = []
                action_taken = []
                print("Greedy episode ongoing")

                state = torch.Tensor(subdata).unsqueeze(0)
                episode_reward = 0
                steps = 0

                state_plot.append([])
                st = state.numpy()[0]
                sta = [st[0], st[1]]
                state_plot[_].append(sta)

                while True:
                    action = agent.select_action(state)
                    a = action.numpy()[0] * 50
                    act_pub = [a[0], a[1]]
                    pub.publish(act_pub)
                    next_state = torch.Tensor(subdata).unsqueeze(0)
                    reward, done, obs_hit = env.calc_shaped_reward(next_state)
                    episode_reward += reward

                    state_visited.append(state)
                    action_taken.append(action)

                    state = next_state

                    steps += 1
                    if done or steps == args.num_steps:
                        greedy_reward.append(episode_reward)
                        break
                    rate.sleep()

                if obs_hit:
                    steps = 300

                steps_to_goal.append(steps)

                # -- plot path --
                if i_episode % 10 == 0:
                    agent.plot_path(state_visited, action_taken, i_episode)


            upper_reward.append((np.max(greedy_reward[-greedy_range:])))
            lower_reward.append((np.min(greedy_reward[-greedy_range:])))
            avg_greedy_reward.append((np.mean(greedy_reward[-greedy_range:])))
            avg_steps_to_goal.append((np.mean(steps_to_goal[-greedy_range:])))


            print("Episode: {}, total numsteps: {}, avg_greedy_reward: {}, average reward: {}".format(
               i_episode, total_numsteps, avg_greedy_reward[-1], np.mean(rewards[-greedy_episode:])))



    #-- saves model --
    if args.save_agent:
        agent.save_model(args.env_name, args.batch_size, i_episode, '.pth')
        with open('exp_replay.pk1', 'wb') as output:
            pickle.dump(memory.memory, output, pickle.HIGHEST_PROTOCOL)
        #with open('exp_replay_g.pk1', 'wb') as output:
        #    pickle.dump(memory_g.memory, output, pickle.HIGHEST_PROTOCOL)

    print('Training ended after {} minutes'.format((time.time() - t_start)/60))
    print('Time per ep : {} s').format((time.time() - t_start) / args.num_episodes)
    print('Mean greedy reward: {}'.format(np.mean(greedy_reward)))
    print('Mean reward: {}'.format(np.mean(rewards)))
    print('Max reward: {}'.format(np.max(rewards)))
    print('Min reward: {}'.format(np.min(rewards)))

    # -- plot learning curve --
    pos_greedy = []
    for pos in range(0, len(lower_reward)):
        pos_greedy.append(pos*greedy_episode)

    plt.title('Greedy policy outcome')
    plt.fill_between(pos_greedy, lower_reward, upper_reward, facecolor='red', alpha=0.3)
    plt.plot(pos_greedy, avg_greedy_reward, 'r')
    plt.xlabel('Number of episodes')
    plt.ylabel('Rewards')
    fname1 = 'plot1_obs_{}_{}_{}'.format(args.env_name, args.batch_size, '.png')
    plt.savefig(fname1)
    plt.close()

    plt.title('Steps to reach goal')
    plt.plot(steps_to_goal)
    plt.ylabel('Number of steps')
    plt.xlabel('Number of episodes')
    fname2 = 'plot2_obs_{}_{}_{}'.format(args.env_name, args.batch_size, '.png')
    plt.savefig(fname2)
    plt.close()


if __name__ == '__main__':
    main()

