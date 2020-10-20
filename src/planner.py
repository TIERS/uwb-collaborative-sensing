#!/usr/bin/env python2
"""
Custom TSP planner

Forked from https://github.com/ctu-mrs/mtsp_planning_task/tree/master/mtsp_problem_loader

"""

import os
import datetime
import itertools
import copy
import numpy as np
import dubins
import random
this_script_path = os.path.dirname(__file__)

# MRS ROS messages
# from mtsp_msgs.msg import TspProblem
# from mrs_msgs.msg import TrajectoryReference 

# the TSP problem class
# from mtsp_problem_loader.tsp_problem import *

from tsp_solvers import *
import tsp_trajectory

import matplotlib.pyplot as plt
from matplotlib.pyplot import cm

import argparse

class Point:

    def __init__(self, x, y) :
        self.x = x
        self.y = y

def p_distance(p1, others):
    d = ((p1.x - others[0].x) ** 2 + (p1.y - others[0].y) ** 2)
    for i in range(1, len(others)) :
        d -= ((p1.x - others[1].x) ** 2 + (p1.y - others[1].y) ** 2)
    return d


class TspPlanner:

    def __init__(self, use_ros=True):

        print("runing as bare script")

        self._plot = True
        self.number_of_robots = 2
        
        
        self._max_velocity = 3
        self._max_acceleration = 1
 
        self._turning_velocity = 1
        
        print("using max_velocity", self._max_velocity)
        print("using max_acceleration", self._max_acceleration)
        print("using plot", self._plot)
        print("using turning_velocity", self._turning_velocity)
                
        # load_problem
        tsp_problem = MTSPProblem.load_problem(args.problem) 
        
        self.plan_trajectory(tsp_problem)

    def plan_trajectory(self, tsp_problem): 
        """method for planning the M(D)TSP(N) plans based on tsp_problem"""
        
        tsp_solver = TSPSolver()

        shuffle_max_trajectory_time = 9999
        shuffle_paths = [[],[]]

        clusters = [[tsp_problem.start_positions[i]] for i in range(tsp_problem.number_of_robots)]  # initiate cluster with starts
        print("clusters with start", clusters)


        starting_positions = [Point(p[0][1], p[0][2]) for p in clusters]
        my_points = [Point(p[1], p[2]) for p in tsp_problem.targets]

        my_ordered_points = sorted(my_points, key=lambda e: p_distance(e, starting_positions))
        tsp_problem_targets = tsp_problem.targets[:]
        for i, p in enumerate(my_ordered_points) :
            tsp_problem_targets[i][1] = p.x
            tsp_problem_targets[i][2] = p.y

        starting_time = datetime.datetime.now()


        final_stop_ids = []

        list_of_possible_stop_ids = list(range(int(len(tsp_problem.targets)/3), int(2*len(tsp_problem.targets)/3) ))

        possible_stop_ids = itertools.combinations(list_of_possible_stop_ids, tsp_problem.number_of_robots-1)

        closer_ones = True
        if closer_ones :

            start_shuffle_id = int(len(tsp_problem_targets))/2-2
            if start_shuffle_id < 0 :
                start_shuffle_id = 0
            stop_shuffle_id = int(len(tsp_problem_targets))/2+1

            middle_tsp_problem_targets = tsp_problem_targets[start_shuffle_id:stop_shuffle_id]
            perms = list(itertools.permutations(middle_tsp_problem_targets))
            print(perms)
            print(len(perms))
            # exit()

            # with open('mydata.txt', 'w') as mydata :

            # for shuffle_trial in range(23) :
            for shuffle_trial, shuffled_middle_tsp_problem_targets in enumerate(itertools.permutations(middle_tsp_problem_targets)) :

                if shuffle_trial > 0 :

                    # shuffled_middle_tsp_problem_targets = tsp_problem_targets[start_shuffle_id:stop_shuffle_id]
                    # random.shuffle(shuffled_middle_tsp_problem_targets)
                    tsp_problem_targets[start_shuffle_id:stop_shuffle_id] = shuffled_middle_tsp_problem_targets
                    for fixing_ids in range(start_shuffle_id, stop_shuffle_id) :
                        tsp_problem_targets[fixing_ids][0] = tsp_problem_targets[0][0]+start_shuffle_id+fixing_ids

                # mydata.write("\n\n\n[{}] tsp_problem_targets {}\n".format(shuffle_trial, tsp_problem_targets))

                possible_stop_ids = itertools.combinations(list_of_possible_stop_ids, tsp_problem.number_of_robots-1)
                # for trial in range(2) :
                for trial, middle_stop_ids in enumerate(possible_stop_ids) :

                    tsp_solver = TSPSolver()

                    print(middle_stop_ids)
                    stop_ids = [m for m in middle_stop_ids]
                    stop_ids.append(len(tsp_problem.targets))
                    # mydata.write("[{}] stop_ids = {}\n".format(trial, stop_ids))


               
                    ############### TARGET LOCATIONS CLUSTERING BEGIN ###############
                    clusters = [[tsp_problem.start_positions[i]] for i in range(tsp_problem.number_of_robots)]  # initiate cluster with starts
                    for i in range(tsp_problem.number_of_robots):
                        start_id = 0#i * len(tsp_problem.targets) / tsp_problem.number_of_robots      
                        if i > 0 :
                            start_id = stop_ids[i-1]  
                        clusters[i] += tsp_problem_targets[start_id:stop_ids[i]]      

                    ############### TARGET LOCATIONS CLUSTERING END ###############
                    

                    # # | ---------------------- solve the TSP --------------------- |
                    robot_sequences = []
                    path = [[]]*tsp_problem.number_of_robots
                    # for i in range(1):
                    for i in range(tsp_problem.number_of_robots):

                        ############### TSP SOLVERS PART BEGIN ###############
                        # path = tsp_solver.plan_tour_etsp(clusters[i],0) #find decoupled ETSP tour over clusters
                        # path = tsp_solver.plan_tour_etspn_decoupled(clusters[i], 0, tsp_problem.neighborhood_radius * 0.8)  # find decoupled ETSPN tour over clusters
                        
                        turning_radius = (self._turning_velocity * self._turning_velocity) / self._max_acceleration
                        path[i] = tsp_solver.plan_tour_dtspn_decoupled(clusters[i], 0, tsp_problem.neighborhood_radius * 0.8, turning_radius)  # find decoupled DTSPN tour over clusters
                        # path = tsp_solver.plan_tour_dtspn_noon_bean(clusters[i], 0, tsp_problem.neighborhood_radius * 0.8, turning_radius) # find noon-bean DTSPN tour over clusters
                        
                        ############### TSP SOLVERS PART END ###############
                        
                        print("path", path[i])
                        robot_sequences.append(path[i])

                    trajectory = tsp_trajectory.TSPTrajectory(self._max_velocity, self._max_acceleration)

                    # # | ------------------- sample trajectories ------------------ |
                    trajectories_samples = []
                    max_trajectory_time = 0
                    for i in range(len(robot_sequences)):

                        if len(robot_sequences[i][0]) == 2 :
                            single_trajectory_samples, trajectory_time = trajectory.sample_trajectory_euclidean(robot_sequences[i])
                        elif len(robot_sequences[i][0]) == 3:
                            single_trajectory_samples, trajectory_time = trajectory.sample_trajectory_dubins(robot_sequences[i], turning_velocity=self._turning_velocity)
                        
                        print("trajectory_time", i, "is", trajectory_time)
                        trajectories_samples.append(single_trajectory_samples)
                        
                        if trajectory_time > max_trajectory_time:
                            max_trajectory_time = trajectory_time
                        
                        
                    # mydata.write("[{}] maximal time of trajectory is {}\n\n".format(trial, max_trajectory_time))

                    #
                    #   Check if new paths are updated
                    #
                    if max_trajectory_time < shuffle_max_trajectory_time :
                        shuffle_max_trajectory_time = max_trajectory_time
                        shuffle_paths = trajectories_samples
                        shuffle_single_trajectory_samples = single_trajectory_samples
                        shuffle_path = path
                        shuffle_clusters = clusters
                        final_stop_ids = stop_ids



        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #
        #


        for trial in range(0) : #range(42) :

            tsp_solver = TSPSolver()

            tsp_problem_targets = tsp_problem.targets[:]
            if trial > 0 :
                random.shuffle(tsp_problem_targets)
        
            ############### TARGET LOCATIONS CLUSTERING BEGIN ###############
            clusters = [[tsp_problem.start_positions[i]] for i in range(tsp_problem.number_of_robots)]  # initiate cluster with starts
            for i in range(tsp_problem.number_of_robots):
                start_id = i * len(tsp_problem.targets) / tsp_problem.number_of_robots
                stop_id = (i + 1) * len(tsp_problem.targets) / tsp_problem.number_of_robots            
                clusters[i] += tsp_problem_targets[start_id:stop_id]      

            ############### TARGET LOCATIONS CLUSTERING END ###############
            
            
            # # | -------------------- plot the clusters ------------------- |
            if self._plot:  # plot the clusters
                colors = cm.rainbow(np.linspace(0, 1, tsp_problem.number_of_robots))
                for i in range(tsp_problem.number_of_robots):
                    # plt.plot([cluster_centers[i][0]],[cluster_centers[i][1]],'*',color=colors[i])
                    plt.plot([c[1] for c in clusters[i]], [c[2] for c in clusters[i]], '.', color=colors[i])

            # # | ---------------------- solve the TSP --------------------- |
            robot_sequences = []
            path = [[]]*tsp_problem.number_of_robots
            # for i in range(1):
            for i in range(tsp_problem.number_of_robots):

                ############### TSP SOLVERS PART BEGIN ###############
                # path = tsp_solver.plan_tour_etsp(clusters[i],0) #find decoupled ETSP tour over clusters
                # path = tsp_solver.plan_tour_etspn_decoupled(clusters[i], 0, tsp_problem.neighborhood_radius * 0.8)  # find decoupled ETSPN tour over clusters
                
                turning_radius = (self._turning_velocity * self._turning_velocity) / self._max_acceleration
                path[i] = tsp_solver.plan_tour_dtspn_decoupled(clusters[i], 0, tsp_problem.neighborhood_radius * 0.8, turning_radius)  # find decoupled DTSPN tour over clusters
                # path = tsp_solver.plan_tour_dtspn_noon_bean(clusters[i], 0, tsp_problem.neighborhood_radius * 0.8, turning_radius) # find noon-bean DTSPN tour over clusters
                
                ############### TSP SOLVERS PART END ###############
                
                print("path", path[i])
                robot_sequences.append(path[i])
                
                # # | -------------------- plot the solution ------------------- |
                if self._plot:  # plot tsp solution
                    sampled_path_all = []
                    for pid in range(1, len(path[i])):
                        if len(path[i][pid]) == 2 :
                            sampled_path_all += path[i]
                            sampled_path_all += [path[i][0]]
                            #plt.plot([path[pid - 1][0] , path[pid][0]], [path[pid - 1][1] , path[pid][1]], '-', color=colors[i], lw=0.8, label='trajectory %d' % (i + 1))
                        elif len(path[i][pid]) == 3 :
                            dubins_path = dubins.shortest_path(path[i][pid - 1], path[i][pid], turning_radius)
                            sampled_path , _ = dubins_path.sample_many(0.1)
                            sampled_path_all += sampled_path
                    
                    plt.plot([p[0] for p in sampled_path_all] , [p[1] for p in sampled_path_all] , '-', color=colors[i], lw=1.2, label='trajectory %d' % (i + 1))

            trajectory = tsp_trajectory.TSPTrajectory(self._max_velocity, self._max_acceleration)

            # # | ------------------- sample trajectories ------------------ |
            trajectories_samples = []
            max_trajectory_time = 0
            for i in range(len(robot_sequences)):

                if len(robot_sequences[i][0]) == 2 :
                    single_trajectory_samples, trajectory_time = trajectory.sample_trajectory_euclidean(robot_sequences[i])
                elif len(robot_sequences[i][0]) == 3:
                    single_trajectory_samples, trajectory_time = trajectory.sample_trajectory_dubins(robot_sequences[i], turning_velocity=self._turning_velocity)
                
                print("trajectory_time", i, "is", trajectory_time)
                trajectories_samples.append(single_trajectory_samples)
                
                if trajectory_time > max_trajectory_time:
                    max_trajectory_time = trajectory_time
                
                if self._plot:  # plot trajectory samples
                    plt.plot([p[0] for p in single_trajectory_samples], [p[1] for p in single_trajectory_samples], 'o', markerfacecolor=colors[i], markeredgecolor='k', ms=2.2, markeredgewidth=0.4 , label='samples %d' % (i + 1))
                    
            if self._plot:  # add legend to trajectory plot
                plt.legend(loc='upper right')
                
            print("maximal time of trajectory is", max_trajectory_time)

            #
            #   Check if new paths are updated
            #
            if max_trajectory_time < shuffle_max_trajectory_time :
                shuffle_max_trajectory_time = max_trajectory_time
                shuffle_paths = trajectories_samples
                shuffle_single_trajectory_samples = single_trajectory_samples
                shuffle_path = path
                shuffle_clusters = clusters

            # # | --------------- plot velocity profiles --------------- |
            if self._plot:  # plot velocity profile
                for i in range(len(trajectories_samples)):
                    trajectory.plot_velocity_profile(trajectories_samples[i], color=colors[i],title = 'Velocity profile %d' % (i + 1))
            
            # # | ----------------------- show plots ---------------------- |
            if self._plot:
                plt.show()


        #
        #
        #   FINAL PLOT
        #
        #
        if self._plot:


            colors = cm.rainbow(np.linspace(0, 1, tsp_problem.number_of_robots))
            for i in range(tsp_problem.number_of_robots):
                # plt.plot([cluster_centers[i][0]],[cluster_centers[i][1]],'*',color=colors[i])
                plt.plot([c[1] for c in shuffle_clusters[i]], [c[2] for c in shuffle_clusters[i]], '.', color=colors[i])

            for i in range(tsp_problem.number_of_robots):
                sampled_path_all = []
                for pid in range(1, len(shuffle_path[i])):
                    if len(shuffle_path[i][pid]) == 2 :
                        sampled_path_all += shuffle_path[i]
                        sampled_path_all += [shuffle_path[i][0]]
                        #plt.plot([path[pid - 1][0] , path[pid][0]], [path[pid - 1][1] , path[pid][1]], '-', color=colors[i], lw=0.8, label='trajectory %d' % (i + 1))
                    elif len(shuffle_path[i][pid]) == 3 :
                        dubins_path = dubins.shortest_path(shuffle_path[i][pid - 1], shuffle_path[i][pid], turning_radius)
                        sampled_path , _ = dubins_path.sample_many(0.1)
                        sampled_path_all += sampled_path
                
                plt.plot([p[0] for p in sampled_path_all] , [p[1] for p in sampled_path_all] , '-', color=colors[i], lw=1.2, label='trajectory %d' % (i + 1))

            for i in range(len(robot_sequences)):
                plt.plot([p[0] for p in shuffle_single_trajectory_samples], [p[1] for p in shuffle_single_trajectory_samples], 'o', markerfacecolor=colors[i], markeredgecolor='k', ms=2.2, markeredgewidth=0.4 , label='samples %d' % (i + 1))
                    
            plt.legend(loc='upper right')
            for i in range(len(shuffle_paths)):
                trajectory.plot_velocity_profile(shuffle_paths[i], color=colors[i],title = 'Velocity profile %d' % (i + 1))
            

        #
        #
        #   RETURN
        #
        #

        elapsed_time = datetime.datetime.now() - starting_time
        print("\n\n\n\n\n\n\n shuffle_paths={}\n\n\n\n\n\n shuffle_max_trajectory_time={}\n\n\n\n\n\n time={}\n\n\n\n\n\n stop_ids={}".format(shuffle_paths, shuffle_max_trajectory_time, elapsed_time, final_stop_ids))
        plt.show()
        return shuffle_paths   
            

if __name__ == '__main__':
    tsp_planner = TspPlanner()
        
