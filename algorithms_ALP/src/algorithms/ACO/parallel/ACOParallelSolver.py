#  MIT License
#
#  Copyright (c) 2022 Matheus Phelipe Alves Pinto
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#  The above copyright notice and this permission notice shall be included in all
#  copies or substantial portions of the Software.
#
#  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#  SOFTWARE.
import multiprocessing
import random
import time
from datetime import datetime

from algorithms_ALP.src.algorithms.ACO.ACOSolver import ACOSolver
import os

from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.algorithms.ACO.entity.Ant import Ant
from algorithms_ALP.src.algorithms.ACO.parallel.ACOIterationExecutor import ACOIterationExecutor


class ACOParallelSolver(ACOSolver):
    def __init__(self, runaway_number, number_of_ants, evaporation_rate, pheromone_rate,
                 alpha=1, beta=0.7, beta1=1, beta2=1):
        super().__init__(runaway_number=runaway_number, number_of_ants=number_of_ants,
                         evaporation_rate=evaporation_rate, pheromone_rate=pheromone_rate,
                         alpha=alpha, beta=beta, beta1=beta1, beta2=beta2)
        self.SIZE = 0
        self.batch_size = 0
        self.max_thread = self.number_of_ants

    def setup(self):
        self.SIZE = len(self.colony)
        # self.batch_size = int(self.SIZE / self.max_thread)
        self.batch_size = self.max_thread
        print("Starting ACO Parallel Solver - v1.0")
        # print(f"Estimated amount of Ant's by batch: {self.batch_size}   Using {self.max_thread} / {int(os.cpu_count())} available threads")
        print(f"Max parallel Ants: {self.batch_size}")

    def start(self, alp_instance: ALPInstance, max_iterations=100):
        self.initialize(alp_instance)
        self.setup()
        global_start = datetime.now()
        thread_executors = []
        for iteration in range(max_iterations):
            counter = 0
            previous = 0
            thread_executors.clear()
            for key, ant in enumerate(self.colony):
                aco_executor = ACOIterationExecutor(key, self.alpha, self.beta1, self.beta2,
                                                    self.pheromone_matrix, self.separation_times_matrix,
                                                    self.global_runaway_dict, self.global_aircraft_candidates,
                                                    self.runaway_indices, self.colony[key])
                thread_executors.append(aco_executor)

            queue_list = []
            for aco_executor in thread_executors:
                if len(queue_list) < self.max_thread:
                    aco_executor.start()
                    queue_list.append(aco_executor)
                else:
                    for executor in queue_list:
                        executor.join()
                        queue_list.remove(executor)
                    aco_executor.start()
                    queue_list.append(aco_executor)

            for executor in queue_list:
                executor.join()

            # for key, ant in enumerate(self.colony):
            #     if counter <self.batch_size:
            #         aco_executor = ACOIterationExecutor(key, self.alpha, self.beta1, self.beta2,
            #                                             self.pheromone_matrix, self.separation_times_matrix,
            #                                             self.global_runaway_dict, self.global_aircraft_candidates,
            #                                             self.runaway_indices, self.colony[key])
            #         #print(f"Thread {aco_executor.id}")
            #         aco_executor.start()
            #         thread_executors.append(aco_executor)
            #         counter +=1
            #     else:
            #         # print("Ant queue busy. Let's wait for a while...")
            #         for executor in thread_executors:
            #             executor.join()
            #         aco_executor = ACOIterationExecutor(key, self.alpha, self.beta1, self.beta2,
            #                                             self.pheromone_matrix, self.separation_times_matrix,
            #                                             self.global_runaway_dict, self.global_aircraft_candidates,
            #                                             self.runaway_indices, self.colony[key])
            #         #print(f"Thread {aco_executor.id}")
            #         aco_executor.start()
            #         thread_executors.append(aco_executor)
            #         counter=1

            self.save_results()
            # print(f"{iteration + 1} Finish iteration: {self.local_glorious_ant.solution_cost}")
            self.update_pheromone_trail(iteration,
                                        best_solution=self.local_glorious_ant.solution_cost <= self.global_glorious_ant.solution_cost)
            self.restart_ants(alp_instance)

        global_finish = datetime.now()
        self.time_execution = global_finish - global_start
        print(f"Finishing algorithm execution: ETA {self.time_execution} seconds")
        print(f"Last Cost solution: {self.local_glorious_ant.solution_cost}")
        print(f"Best solution: {self.global_glorious_ant.solution_cost}")
        x = 0

    def save_results(self):
        # time.sleep(1)
        self.local_glorious_ant = min(self.colony, key=lambda ant: ant.solution_cost)
        # print("---------")
        # for key, ant in enumerate(self.colony):
        #    print(key, ant.solution_cost)
        # print(f"selected min:  {self.local_glorious_ant.solution_cost}")
        if self.global_glorious_ant is None:
            self.global_glorious_ant = self.local_glorious_ant
        elif self.local_glorious_ant.solution_cost < self.global_glorious_ant.solution_cost:
            self.global_glorious_ant = self.local_glorious_ant
        self.iterations_costs.append(self.local_glorious_ant.solution_cost)

    def restart_ants(self, alp_instance):
        """
        Provides a battalion of ants to build the solutions.
        :param alp_instance:
        """
        # Start colony with blank data (alzheimer's crisys)
        self.colony.clear()
        for ant_id in range(self.number_of_ants):
            self.colony.append(Ant(alp_instance, self.runaway_indices, self.aircraft_indices))
