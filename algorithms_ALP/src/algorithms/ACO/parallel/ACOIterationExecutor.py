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
import random
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
                 pheromone_matrix, separation_times_matrix, global_runway_dict, global_aircraft_candidates,
                 runway_indices, ant: Ant):
        super().__init__()
        self.id = id
        self.alpha = alpha
        self.beta1 = beta1
        self.beta2 = beta2
        self.pheromone_matrix = pheromone_matrix
        self.separation_times_matrix = separation_times_matrix
        self.global_runway_dict = global_runway_dict
        self.global_aircraft_candidates = global_aircraft_candidates
        self.runway_indices = runway_indices
        self.ant = ant

    def run2(self):
        print(f"Hello from ant {self.id}")

    def run(self) -> None:
        # print(f"Start ant id {self.id} cost {self.ant.solution_cost}")
        while len(self.ant.aircraft_candidates_dict) > 0:
            selected_runaway: Runway = self.select_runway(self.ant)
            selected_aircraft = self.select_aircraft(self.ant, selected_runaway)

            priority = self.get_aircraft_priority(selected_aircraft.index, sel=1)
            self.ant.update_heuristic_info(selected_runaway, self.beta1, self.beta2, priority)

            self.ant.aircraft_candidates_dict.pop(selected_aircraft.index)
            ant_runaway: Runway = self.ant.runaways_dict[selected_runaway.index]
            ant_runaway.solution_dict[selected_aircraft.index] = selected_aircraft
        self.ant.compute_total_costs()

    def select_runway(self, ant: Ant):
        """
        For ant k, there is a probability rule to select a runway r, from node D.
        :return: Runaway
        """
        q0 = 0.5  # 0< q0 < 1 is a constant of the algorithm
        q = random.uniform(0, 1)
        r0 = self.global_runway_dict[random.choice(self.runway_indices)]
        if q < q0:
            runaway_cal_list = []
            for key, runaway in ant.runaways_dict.items():
                if len(runaway.solution_dict) > 0:
                    last_plane = runaway.solution_dict[
                        list(runaway.solution_dict.keys())[-1]]
                    last_time_plus_sep_time_list = []
                    for key, candidate_aircraft in ant.aircraft_candidates_dict.items():
                        separation_time = self.separation_times_matrix[last_plane.aircraft_id][
                            candidate_aircraft.aircraft_id]
                        last_time_plus_sep_time_list.append(last_plane.landing_time + separation_time)
                    runaway_cal_list.append({'runaway_index': runaway.index, 'time': min(last_time_plus_sep_time_list)})
                else:
                    return r0

            better_runaway = runaway_cal_list[0]
            for runaway_aux in runaway_cal_list:
                if runaway_aux['time'] <= better_runaway['time']:
                    better_runaway = runaway_aux
            return ant.runaways_dict[better_runaway['runaway_index']]

        return r0

    def select_aircraft(self, ant: Ant, runaway: Runway):
        """
        After choosing a runway r, the ant has to choose an aircraft for this runway.
        :param ant:
        :param runaway:
        :return:
        """
        prob_list = []
        for key, aircraft in ant.aircraft_candidates_dict.items():
            if len(ant.aircraft_candidates_dict.items()) > 1:
                aircraft.landing_time = self.assign_landing_time_to_aircraft(ant, runaway, aircraft)
                aircraft.penality_cost_computed = ACOUtils.evaluate_cost(aircraft)
                aircraft.probability_of_choose = self.compute_probability(ant, runaway, aircraft)
                prob_list.append(aircraft.probability_of_choose)

            else:
                aircraft.landing_time = self.assign_landing_time_to_aircraft(ant, runaway, aircraft)
                aircraft.penality_cost_computed = self.evaluate_cost(aircraft)
                return aircraft

        return ant.aircraft_candidates_dict[
            MathUtils.choice_from_probability(np.array(list(ant.aircraft_candidates_dict.keys())),
                                              prob_distribution=np.array(prob_list))]

    def assign_landing_time_to_aircraft(self, ant: Ant, selected_runway: Runway, selected_aircraft: Aircraft):
        """
        This step is to assign landing times to aircraft while respecting the two constraints:
            − The landing time must be within the landing widow [ei, li]
            − The interval of security must be respected
        :param ant: current Ant
        :param selected_runway:
        :param selected_aircraft:
        :return:
        """
        runaway_solutions = ant.runaways_dict[selected_runway.index]
        aircraft_times = [selected_aircraft.target_landing_time]
        aux_aicraft_times = []
        for key, landed_air in runaway_solutions.solution_dict.items():
            landed_aircraft: Aircraft = landed_air

            separation_time = self.separation_times_matrix[landed_aircraft.aircraft_id][selected_aircraft.aircraft_id]
            aux_aicraft_times.append(landed_aircraft.landing_time + separation_time)
        if len(aux_aicraft_times) > 0:
            aircraft_times.append(max(aux_aicraft_times))

        return max(aircraft_times)

    def compute_probability(self, ant: Ant, runaway: Runway, aircraft: Aircraft):
        """
        Evaluate the probability rule to choose an aircraft to landing on selected runaway.
        :param aircraft:
        :param runaway:
        :param ant:
        """
        try:
            try:
                numerator = (self.pheromone_matrix[runaway.index][aircraft.index] ** self.alpha) * \
                            ant.heuristic_info[runaway.index][aircraft.index] ** ((self.beta1 + self.beta2) / 100)
                denominator = 0
                for key, aircraft_unvisited in ant.aircraft_candidates_dict.items():
                    denominator += self.pheromone_matrix[runaway.index][aircraft_unvisited.index] ** self.alpha * \
                                   ant.heuristic_info[runaway.index][aircraft_unvisited.index] ** (
                                               (self.beta1 + self.beta2) / 100)

                return numerator / denominator

            except Exception as ex:
                pass

        except Exception as ex:
            pass

        return 0

    def evaluate_cost(self, aircraft: Aircraft):
        """
        Apply the objective function from problem formulation to collect all deviation costs from each aircraft.
        :param aircraft:
        :param ant:
        """
        deviation_time = aircraft.landing_time - aircraft.target_landing_time
        if deviation_time > 0:
            return aircraft.penality_cost_latest * deviation_time

        return aircraft.penality_cost_earliest * abs(deviation_time)

    def get_aircraft_priority(self, aircraft_index, sel=7):
        aircraft: Aircraft = self.global_aircraft_candidates[aircraft_index]

        if sel == 1:    #
            return aircraft.appearance_time
        elif sel == 2:  # ei
            return aircraft.earliest_landing_time
        elif sel == 3:  # tai
            return aircraft.target_landing_time
        elif sel == 4:  # li
            return aircraft.latest_landing_time
        elif sel == 5:   # ei / Pbi
            return aircraft.earliest_landing_time / aircraft.penality_cost_earliest
        elif sel == 6:  # li / Pai
            return aircraft.latest_landing_time / aircraft.penality_cost_latest
        elif sel == 7:  # tai / (Pbi + Pai) TESTANDO
            return aircraft.target_landing_time / (aircraft.penality_cost_earliest + aircraft.penality_cost_latest)
        elif sel == 8:  # li / (Pbi + Pai)
            return aircraft.latest_landing_time / (aircraft.penality_cost_earliest + aircraft.penality_cost_latest)
        elif sel == 9:
            return 2*aircraft.target_landing_time / (aircraft.penality_cost_earliest + aircraft.penality_cost_latest)
