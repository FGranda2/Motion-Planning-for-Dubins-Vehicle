import copy
import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import AutoMinorLocator
from matplotlib import gridspec


def main():
    cost_rrt = np.loadtxt('data_rrt2.csv', delimiter=',')
    cost_rrt_star = np.loadtxt('data_rrt_star2.csv', delimiter=',')
    #binwidth = 1
    #plt.hist(cost_rrt, bins=range(int(np.rint(min(cost_rrt))), int(np.rint(max(cost_rrt)) + binwidth), binwidth),
    #         alpha=0.5, label='RRT')
    #plt.hist(cost_rrt_star,
    #         bins=range(int(np.rint(min(cost_rrt_star))), int(np.rint(max(cost_rrt_star)) + binwidth), binwidth),
    #         alpha=0.5, label='RRT*')
    #plt.legend(loc='upper right')
    #plt.show()
    binwidth = 5
    fig, ax = plt.subplots(1, figsize=(16, 6))
    n, bins, patches = plt.hist(cost_rrt, bins=range(int(np.rint(min(cost_rrt))), int(np.rint(max(cost_rrt)) + binwidth), binwidth),
             alpha=0.5, label='RRT')
    # define minor ticks and draw a grid with them
    #minor_locator = AutoMinorLocator(2)
    #plt.gca().xaxis.set_minor_locator(minor_locator)
    #plt.grid(which='minor', color='white', lw=0.5)
    # x ticks
    xticks = [(bins[idx + 1] + value) / 2 for idx, value in enumerate(bins[:-1])]
    #xticks_labels = ["{:.2f}\nto\n{:.2f}".format(value, bins[idx + 1]) for idx, value in enumerate(bins[:-1])]
    #plt.xticks(xticks, labels=xticks_labels)
    #ax.tick_params(axis='x', which='both', length=0)
    # remove y ticks
    #plt.yticks([])
    # Hide the right and top spines
    #ax.spines['bottom'].set_visible(False)
    #ax.spines['left'].set_visible(False)
    #ax.spines['right'].set_visible(False)
    #ax.spines['top'].set_visible(False)
    # plot values on top of bars
    for idx, value in enumerate(n):
        if value > 0:
            plt.text(xticks[idx], value + 5, int(value), ha='center')
    #plt.title('Path Length of RRT', loc='center', fontsize=18)
    #plt.show()

    # fig, ax = plt.subplots(1, figsize=(16, 6))
    n, bins, patches = plt.hist(cost_rrt_star,
                                bins=range(int(np.rint(min(cost_rrt_star))), int(np.rint(max(cost_rrt_star)) + binwidth),
                                           binwidth),
                                alpha=0.5, label='RRT*')
    # define minor ticks and draw a grid with them
    #minor_locator = AutoMinorLocator(2)
    #plt.gca().xaxis.set_minor_locator(minor_locator)
    #plt.grid(which='minor', color='white', lw=0.5)
    # x ticks
    xticks = [(bins[idx + 1] + value) / 2 for idx, value in enumerate(bins[:-1])]
    #xticks_labels = ["{:.2f}\nto\n{:.2f}".format(value, bins[idx + 1]) for idx, value in enumerate(bins[:-1])]
    #plt.xticks(xticks)
    #ax.tick_params(axis='x', which='both', length=0)
    # remove y ticks
    # plt.yticks([])
    # Hide the right and top spines
    # ax.spines['bottom'].set_visible(False)
    # ax.spines['left'].set_visible(False)
    # ax.spines['right'].set_visible(False)
    # ax.spines['top'].set_visible(False)
    # plot values on top of bars
    for idx, value in enumerate(n):
        if value > 0:
            plt.text(xticks[idx], value + 5, int(value), ha='center')
    plt.title('Path Lengths Comparison RRT vs RRT* (Bin Size=5)', loc='center', fontsize=18)
    plt.xlabel('Path Length')
    plt.ylabel('Instances')
    plt.xticks(range(0,130,5))
    plt.legend(loc='upper right')
    plt.show()
    print('The mean for RRT is:', np.mean(cost_rrt))
    print('The mean for RRT* is:', np.mean(cost_rrt_star))
    print('The std for RRT is:', np.nanstd(cost_rrt))
    print('The std for RRT* is:', np.nanstd(cost_rrt_star))

if __name__ == '__main__':
    main()