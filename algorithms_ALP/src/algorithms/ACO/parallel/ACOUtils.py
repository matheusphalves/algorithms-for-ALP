import random
import numpy as np

from algorithms_ALP.src.algorithms.ACO.entity.Aircraft import Aircraft
from algorithms_ALP.src.algorithms.ACO.entity.Ant import Ant
from algorithms_ALP.src.algorithms.ACO.entity.Runway import Runway
from algorithms_ALP.src.utils.math.MathUtils import MathUtils


class ACOUtils:

    @staticmethod
    def select_runway(ant: Ant, global_runway_dict, runway_indices, separation_times_matrix):
        """
        For ant k, there is a probability rule to select a runway r, from node D.
        :return: Runaway
        """
        q0 = 0.5  # 0< q0 < 1 is a constant of the algorithm
        q = random.uniform(0, 1)
        r0 = global_runway_dict[random.choice(runway_indices)]
        if q < q0:
            runaway_cal_list = []
            for key, runaway in ant.runaways_dict.items():
                if len(runaway.solution_dict) > 0:
                    last_plane = runaway.solution_dict[
                        list(runaway.solution_dict.keys())[-1]]
                    last_time_plus_sep_time_list = []
                    for key, candidate_aircraft in ant.aircraft_candidates_dict.items():
                        separation_time = separation_times_matrix[last_plane.aircraft_id][
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

    @staticmethod
    def select_aircraft(ant: Ant, runaway: Runway, separation_times_matrix, pheromone_matrix, alpha, beta1, beta2):
        """
        After choosing a runway r, the ant has to choose an aircraft for this runway.
        :param ant:
        :param runaway:
        :return:
        """
        prob_list = []
        for key, aircraft in ant.aircraft_candidates_dict.items():
            if len(ant.aircraft_candidates_dict.items()) > 1:
                aircraft.landing_time = ACOUtils.assign_landing_time_to_aircraft(ant, runaway, aircraft, separation_times_matrix)
                aircraft.penality_cost_computed = ACOUtils.evaluate_cost(aircraft)
                aircraft.probability_of_choose = ACOUtils.compute_probability(ant, runaway, aircraft, pheromone_matrix, alpha, beta1, beta2)
                prob_list.append(aircraft.probability_of_choose)

            else:
                aircraft.landing_time = ACOUtils.assign_landing_time_to_aircraft(ant, runaway, aircraft, separation_times_matrix)
                aircraft.penality_cost_computed = ACOUtils.evaluate_cost(aircraft)
                return aircraft

        return ant.aircraft_candidates_dict[
            MathUtils.choice_from_probability(np.array(list(ant.aircraft_candidates_dict.keys())),
                                              prob_distribution=np.array(prob_list))]

    @staticmethod
    def assign_landing_time_to_aircraft(ant: Ant, selected_runway: Runway, selected_aircraft: Aircraft, separation_times_matrix):
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

            separation_time = separation_times_matrix[landed_aircraft.aircraft_id][selected_aircraft.aircraft_id]
            aux_aicraft_times.append(landed_aircraft.landing_time + separation_time)
        if len(aux_aicraft_times) > 0:
            aircraft_times.append(max(aux_aicraft_times))

        return max(aircraft_times)

    @staticmethod
    def get_aircraft_priority(aircraft_index, global_aircraft_candidates, sel=7):
        aircraft: Aircraft = global_aircraft_candidates[aircraft_index]

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

    @staticmethod
    def compute_probability(ant: Ant, runaway: Runway, aircraft: Aircraft, pheromone_matrix, alpha, beta1, beta2):
        """
        Evaluate the probability rule to choose an aircraft to landing on selected runaway.
        :param aircraft:
        :param runaway:
        :param ant:
        """
        try:
            try:
                numerator = (pheromone_matrix[runaway.index][aircraft.index] ** alpha) * \
                            ant.heuristic_info[runaway.index][aircraft.index] ** ((beta1 + beta2) /100)
                denominator = 0
                for key, aircraft_unvisited in ant.aircraft_candidates_dict.items():
                    denominator += pheromone_matrix[runaway.index][aircraft_unvisited.index]**alpha * \
                                       ant.heuristic_info[runaway.index][aircraft_unvisited.index]**((beta1 + beta2) /100)

                return numerator / denominator

            except Exception as ex:
                pass

        except Exception as ex:
            pass

        return 0

    @staticmethod
    def evaluate_cost(aircraft: Aircraft):
        """
        Apply the objective function from problem formulation to collect all deviation costs from each aircraft.
        :param aircraft:
        :param ant:
        """
        deviation_time = aircraft.landing_time - aircraft.target_landing_time
        if deviation_time > 0:
            return aircraft.penality_cost_latest * deviation_time

        return aircraft.penality_cost_earliest * abs(deviation_time)