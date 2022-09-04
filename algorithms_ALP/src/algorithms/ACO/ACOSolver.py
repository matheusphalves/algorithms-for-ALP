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

from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.algorithms.ACO.entity.Ant import Ant
from algorithms_ALP.src.algorithms.ACO.entity.Aircraft import Aircraft
from algorithms_ALP.src.algorithms.ACO.entity.Runaway import Runaway
from algorithms_ALP.src.utils.math.MathUtils import MathUtils

import numpy as np
import random
from datetime import datetime


class ACOSolver:
    """
    Ant Colony Optimization for Aircraft Landing Problem.
    """

    def __init__(self, runaway_number, number_of_ants, evaporation_rate, pheromone_intensity,
                 beta_evaporation_rate, alpha=1, beta = 1, beta1=1, beta2=1):
        """
        :param runaway_number: amount of runways available
        :param number_of_ants: amount of Ants to build solutions
        :param evaporation_rate: rate at which pheromone evaporates
        :param pheromone_intensity: constant added to the best path
        :param beta_evaporation_rate: rate at which beta decays (optional)
        :param alpha: weighting of pheromone
        :param beta: weighting of heuristic (visibility of ants)
        :param beta1: weighting of heuristic (priority)
        :param beta2: weighting of heuristic (cost penality)
        """

        # Configurable parameters
        self.runaway_number = runaway_number
        self.number_of_ants = number_of_ants
        self.evaporation_rate = evaporation_rate
        self.pheromone_rate = pheromone_intensity
        self.beta_evaporation_rate = beta_evaporation_rate
        self.alpha = alpha
        self.beta = beta
        self.beta1 = beta1
        self.beta2 = beta2
        self.separation_times_matrix = None

        # Internal parameters
        self.colony = []
        self.global_runaway_dict = {}
        self.global_aircraft_candidates = {}
        self.pheromone_matrix = None
        self.heuristic_info = None
        self.runaway_indices = []
        self.aircraft_indices = []

    def __initialize(self, alp_instance: ALPInstance = None):
        """
        Initialize internal parameters of ACO.
        :param alp_instance:
        :return:
        """

        try:
            # Create pheromony matrix (considering runaways and dummy nodes)
            matrix_dimension = self.runaway_number + len(
                alp_instance.aircraft_times) + 2  # (2) Dummy nodes called D and F
            self.pheromone_matrix, self.runaway_indices, self.aircraft_indices = self.create_matrix_list(
                matrix_dimension, len(alp_instance.aircraft_times))

            # Create global runaway list
            for run_index in self.runaway_indices:
                self.global_runaway_dict[run_index] = Runaway(run_index, runaway_name=f'R{int(run_index)}')

            # Create global aircraft candidate list with index
            for index_plane, airplane_data in alp_instance.aircraft_times.items():
                aux_index_plane = self.aircraft_indices[int(index_plane)]
                self.global_aircraft_candidates[aux_index_plane] = Aircraft(int(aux_index_plane), airplane_data)

            # Start colony with initial data (there are not any solution)
            for ant_id in range(self.number_of_ants):
                self.colony.append(Ant(ant_id=ant_id,
                                       aircraft_candidates_dict=self.global_aircraft_candidates,
                                       runaways_dict=self.global_runaway_dict))

            # Create heuristic info matrix
            self.heuristic_info = np.ones((matrix_dimension, matrix_dimension))

            # Load separation times matrix
            self.separation_times_matrix = alp_instance.separation_times_matrix


        except Exception as ex:
            raise ex

    def start(self, alp_intance: ALPInstance, max_iterations=10):
        self.__initialize(alp_intance)
        global_start = datetime.now()
        for iteration in range(max_iterations):
            iter_start = datetime.now()
            print(f"Starting iteration {int(iteration + 1)} / {max_iterations} expected")
            for ant in self.colony:
                while len(ant.aircraft_candidates_dict) > 0:
                    selected_runaway = self.select_runaway(ant)
                    selected_aircraft = self.select_aircraft(ant, selected_runaway)
                    landing_time = self.assign_landing_time_to_aircraft(ant, selected_runaway, selected_aircraft)
                    # Insert the aircraft j in the list of aircraft affected to the runway r and delete it from the candidate list
                # Return to the beginning of the graph
            self.update_pheromone_trail()

            iter_finish = datetime.now()
            print(f"Finish iteration [{int(iteration)}]: Elapsed {iter_finish - iter_start} seconds")

        global_finish = datetime.now()
        print(f"Finish iteration [{int(iteration)}]: Elapsed {global_finish - global_start} seconds")

    def create_matrix_list(self, matrix_dimension, planes_size):
        """
        Create the adjacency matrix that represents Graph's path for ALP.
        :param matrix_dimension: square dimension (containing dummy nodes)
        :param planes_size: amount of planes
        :return: ndarray
        """
        matrix = []

        # Create connections between dummy node D and runways
        aux_list = [0]
        aux_list = MathUtils.join_lists(aux_list, [1] * self.runaway_number)
        aux_list = MathUtils.join_lists(aux_list, [0] * (planes_size + 1))
        runaway_indices = [i for i, x in enumerate(aux_list) if x == 1]
        aircraft_indices = [i for i, x in enumerate(aux_list) if x == 0][1:-1]
        matrix.append(aux_list)

        # Create connections between runaways and aircrafts
        for runaway in range(self.runaway_number):
            aux_list = [0]
            aux_list = MathUtils.join_lists(aux_list, [0] * self.runaway_number)
            aux_list = MathUtils.join_lists(aux_list, [1] * planes_size)
            aux_list.append(0)
            matrix.append(aux_list)

        # Create connections between aircrafts dummy node F
        for runaway in range(planes_size):
            aux_list = [0]
            aux_list = MathUtils.join_lists(aux_list, [0] * (self.runaway_number + planes_size))
            aux_list.append(1)
            matrix.append(aux_list)
        # Create connection between nodes F and D
        aux_list = [1]
        matrix.append(MathUtils.join_lists(aux_list, [0] * (matrix_dimension - 1)))

        np_matrix = MathUtils.matrix_list_to_np_array(matrix)
        assert np_matrix.shape[0] == np_matrix.shape[1] and np_matrix.shape[
            0] == matrix_dimension, "Matrix dimensions wrong!"

        return np_matrix, runaway_indices, aircraft_indices

    def select_runaway(self, ant: Ant):
        """
        For ant k, there is a probability rule to select a runway r, from node D.
        :return: Runaway
        """
        q0 = 5  # 0< q0 < 1 is a constant of the algorithm
        q = random.randint(0, 1)
        r0 = random.choice(self.global_runaway_dict)
        if q < q0:
            runaway_cal_list = []
            for key, runaway in ant.runaways_dict.items():
                if len(runaway.solution_list) > 0:
                    last_plane = runaway.solution_list[-1]  # last aircraft affected to the runway r for the ant
                    last_time_plus_sep_time_list = []
                    for candidate_index in ant.aircraft_candidates_dict:
                        # Separation time between the landings of aircraft i and j if they land on the same runaway
                        separation_time = self.separation_times_matrix[last_plane.index][candidate_index]
                        last_time_plus_sep_time_list.append(last_plane.landing_time + separation_time)

                    runaway_cal_list.append(min(last_time_plus_sep_time_list))
                else:
                    return r0

            return min(runaway_cal_list)

        return r0

    def select_aircraft(self, ant: Ant, runaway: Runaway):
        """
        After choosing a runway r, the ant has to choose an aircraft for this runway.
        :param ant: 
        :param runaway: 
        :return: 
        """
        for key, aircraft in ant.aircraft_candidates_dict.items():
            aircraft.landing_time = self.assign_landing_time_to_aircraft(ant, runaway, aircraft)
            aircraft.penality_cost_computed = self.evaluate_cost(aircraft)
            self.compute_heuristic_info(runaway, aircraft)
            aircraft.probability_of_choose = self.compute_probability(runaway, aircraft)

        return max(ant.aircraft_candidates_dict, lambda x: x.probability_of_choose)

    def evaluate_cost(self, aircraft: Aircraft):
        """
        Apply the objective function from problem formulation to collect all deviation costs from each aircraft.
        :param ant:
        """
        deviation_time = aircraft.landing_time - aircraft.target_landing_time
        if deviation_time > 0:
            return aircraft.penality_cost_latest * deviation_time

        return aircraft.penality_cost_earliest * abs(deviation_time)

    def compute_probability(self, runaway: Runaway, aircraft: Aircraft):
        """
        Evaluate the probability rule to choose an aircraft to landing on selected runaway.
        :param aircraft:
        :param runaway:
        :param ant:
        """

        try:
            #todo check if candidate list contains aircraft j!!!!!
            numerator = (self.pheromone_matrix[runaway.index][aircraft.index]**self.alpha) * self.heuristic_info[runaway.index][aircraft.index]**self.beta
            denominator = np.sum(np.dot(self.pheromone_matrix**self.alpha, self.heuristic_info**self.beta))
            np.sum(self.pheromone_matrix[
                                        self.runaway_indices[0]:self.runaway_indices[-1]+1,
                                        self.aircraft_indices[0]:self.aircraft_indices[-1] + 1,
                                        ],
                                        self.heuristic_info[
                                        self.runaway_indices[0]:self.runaway_indices[-1]+1,
                                        self.aircraft_indices[0]:self.aircraft_indices[-1] + 1,
                                        ])

            return numerator / denominator
        except Exception as ex:
            pass

        return 0


    def assign_landing_time_to_aircraft(self, ant: Ant, selected_runaway: Runaway, selected_aircraft: Aircraft):
        """
        This step is to assign landing times to aircraft while respecting the two constraints:
            − The landing time must be within the landing widow [ei, li]
            − The interval of security must be respected
        :param ant: current Ant
        :param selected_runaway:
        :param selected_aircraft:
        :return:
        """
        runaway_solutions = ant.runaways_dict[selected_runaway.index]
        aircraft_times = [selected_aircraft.target_landing_time]
        aux_aicraft_times = []
        for landed_air in runaway_solutions.solution_list: # return Runaway list
            landed_aircraft: Aircraft = landed_air

            separation_time = self.separation_times_matrix[landed_aircraft.index][selected_aircraft.index]
            aux_aicraft_times.append(landed_aircraft.earliest_landing_time + separation_time)
        if len(aux_aicraft_times) > 0:
            aircraft_times.append(max(aux_aicraft_times))

        return max(aircraft_times)


    #----- Future works

    def update_pheromone_trail(self):
        """
        The pheromone trail must be updated at the end of each iteration.
        :return:
        """
        pass

    def increase_pheromone_trail(self):
        pass

    def decrease_pheromone_trail(self):
        pass

    def get_aircraft_priority(self, aircraft_index, sel=1):
        aircraft: Aircraft = self.global_aircraft_candidates[aircraft_index]

        if sel == 1:
            return aircraft.appearance_time
        elif sel == 2:
            return aircraft.earliest_landing_time
        elif sel == 3:
            return aircraft.target_landing_time
        else:
            return aircraft.latest_landing_time

    def compute_heuristic_info(self, runaway: Runaway, aircraft: Aircraft):
        """
        A weighting of these two parameters (Priority(i) and penalty_cost(i)) corresponds to the heuristic information.
        :param runaway:
        :param aircraft:
        :return: float
        """
        priority = self.get_aircraft_priority(aircraft.index, sel=1)
        cost_penality = aircraft.penality_cost_computed
        self.heuristic_info[runaway.index][aircraft.index] = (1 / (priority + 1))**self.beta1 * \
               (1 / (cost_penality + 1))**self.beta2 # avoid division by zero
