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
from algorithms_ALP.src.algorithms.ACO.entity.Ant import Ant
from algorithms_ALP.src.utils.math.GraphViewer import GraphViewer


class ACOGraphViewer:

    def __init__(self):
        self.graphs = []

    def visualize_best_solution(self, glorious_ant: Ant):
        for run_index, (runaway_index, runaway) in enumerate(glorious_ant.runaways_dict.items()):
            aco_graph = GraphViewer()
            solution_list = [aircraft_index for air_index, (aircraft_index, aircraft) in enumerate(runaway.solution_dict.items())]
            grouped_sol_list = [solution_list[n:n + 2] for n in range(0, len(solution_list), 1)][:-1]
            aco_graph.add_weigthed_edge(f'R{run_index + 1}', f'A{grouped_sol_list[0][0]}', 0)
            for node in grouped_sol_list:
                aco_graph.add_weigthed_edge(f'A{node[0]}', f'A{node[1]}', 0)

            self.graphs.append(aco_graph)

        for graph in self.graphs:
            graph.visualize_digraph()
