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
from random import random
from threading import Thread
from multiprocessing import Process

import numpy as np

from algorithms_ALP.src.algorithms.ACO.entity.Aircraft import Aircraft
from algorithms_ALP.src.algorithms.ACO.entity.Ant import Ant
from algorithms_ALP.src.algorithms.ACO.entity.Runway import Runway
from algorithms_ALP.src.algorithms.ACO.parallel.ACOUtils import ACOUtils
from algorithms_ALP.src.utils.math.MathUtils import MathUtils


class ACOIterationExecutor(Thread):
    def __init__(self, id, alpha, beta1, beta2,
                 pheromone_matrix, separation_times_matrix, global_runway_dict, global_aircraft_candidates, runway_indices):
        super().__init__()
        self.id = id
        self.local_colony = []
        self.alpha = alpha
        self.beta1 = beta1
        self.beta2 = beta2
        self.pheromone_matrix = pheromone_matrix
        self.separation_times_matrix = separation_times_matrix
        self.global_runway_dict = global_runway_dict
        self.global_aircraft_candidates = global_aircraft_candidates
        self.runway_indices = runway_indices
        # self.queue_result = queue_result

    def run(self) -> None:
        for key_ant, ant in enumerate(self.local_colony):
            while len(ant.aircraft_candidates_dict) > 0:
                selected_runaway: Runway = ACOUtils.select_runway(ant, self.global_runway_dict, self.runway_indices,
                                                                  self.separation_times_matrix)
                selected_aircraft = ACOUtils.select_aircraft(ant, selected_runaway, self.separation_times_matrix, self.pheromone_matrix, self.alpha, self.beta1, self.beta2)

                priority = ACOUtils.get_aircraft_priority(selected_aircraft.index, self.global_aircraft_candidates, sel=1)
                ant.update_heuristic_info(selected_runaway, self.beta1, self.beta2, priority)

                ant.aircraft_candidates_dict.pop(selected_aircraft.index)
                ant_runaway: Runway = ant.runaways_dict[selected_runaway.index]
                ant_runaway.solution_dict[selected_aircraft.index] = selected_aircraft
            ant.compute_total_costs()
            # print(f"[Thread {self.id}]: Ant [{key_ant + 1}] cost: {ant.solution_cost}")
        # self.queue_result.put(self.local_colony)
    # def join(self):
    #     return self.local_colony
