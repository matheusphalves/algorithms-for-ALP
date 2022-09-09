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
from collections import OrderedDict

from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.algorithms.ACO.entity.Aircraft import Aircraft
from algorithms_ALP.src.algorithms.ACO.entity.Runaway import Runaway
from algorithms_ALP.src.utils.math.MathUtils import MathUtils


class Ant:
    """
        Represents an Ant applied to ALP.
    """

    def __init__(self, alp_instance, runaway_indices, aircraft_indices):
        """
        :param plane_candidates_list: A candidate list according to the ant constructs its solution
        :param runaways_dict: lists representing each a runway: it contains both the
            indexes of aircrafts affected to and their landing times
        :param penality_cost: Penalty cost of the solution represented
        """
        self.solution_cost = 0
        self.aircraft_candidates_dict = {}
        self.runaways_dict = {}  # also called as solution_dict
        self.path_matrix = None
        self.initialize_parameters(alp_instance, runaway_indices, aircraft_indices)

    def initialize_parameters(self, alp_instance: ALPInstance, runaway_indices, aircraft_indices):
        # Create global runaway list
        for run_index in runaway_indices:
            self.runaways_dict[run_index] = Runaway(run_index, runaway_name=f'R{int(run_index)}',
                                                    solution_dict=OrderedDict())

        # Create global aircraft candidate list with index
        for index_plane, airplane_data in alp_instance.aircraft_times.items():
            aux_index_plane = aircraft_indices[int(index_plane)]
            self.aircraft_candidates_dict[aux_index_plane] = Aircraft(aircraft_id=int(index_plane),
                                                                      index=int(aux_index_plane), data=airplane_data)

    def compute_total_costs(self):
        solution_cost = 0
        for key, runaway in self.runaways_dict.items():
            runaway.compute_landing_costs()
            solution_cost += runaway.runaway_cost
        self.solution_cost = solution_cost  # avoid multiple sums
        return self.solution_cost
