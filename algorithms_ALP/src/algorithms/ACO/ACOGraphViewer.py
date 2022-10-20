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

from algorithms_ALP.src.algorithms.ACO.entity.Ant import Ant
from algorithms_ALP.src.utils.math.GraphViewer import GraphViewer
import matplotlib.pyplot as plt
import networkx as nx


class ACOGraphViewer:

    def __init__(self):
        self.graphs = []

    def visualize_best_solution(self, glorious_ant: Ant, instance_name):
        for run_index, (runaway_index, runaway) in enumerate(glorious_ant.runaways_dict.items()):
            aco_graph = GraphViewer()
            solution_list = [aircraft_index for air_index, (aircraft_index, aircraft) in enumerate(runaway.solution_dict.items())]
            grouped_sol_list = [solution_list[n:n + 2] for n in range(0, len(solution_list), 1)][:-1]
            aco_graph.add_weigthed_edge(f'R{run_index + 1}', f'A{grouped_sol_list[0][0]-2}', 0)
            for node in grouped_sol_list:
                aco_graph.add_weigthed_edge(f'A{node[0]-2}', f'A{node[1]-2}')

            self.set_layout_postions(aco_graph, solution_list, instance_name)
            self.graphs.append(aco_graph)

        for graph in self.graphs:
            #graph.visualize_digraph()
            #self.visualize_digraph_alp_layout(graph)
            pass


    def set_layout_postions(self, graph, solution_list, instance_name, max_column_group = 3):
        graph.graph = nx.DiGraph()
        graph.graph.add_weighted_edges_from(graph.visual)
        nodes = list(graph.graph.nodes)
        solution_list = sorted(solution_list)
        pos = {}
        pos[nodes[0]] = (0, 1)
        row_counter = 1
        column_counter = 2
        for node in nodes[1:]:
            if column_counter <= max_column_group:
                pos[node] = (column_counter, row_counter)
                column_counter +=1
            else:
                pos[node] = (column_counter, row_counter)
                row_counter +=1
                column_counter = 1
        nx.draw_networkx(graph.graph, pos)
        plt.title(f"ACO - Scheduled sequence {instance_name}\nrunway {nodes[0]} - ({(len(solution_list))} aircrafts)")
        plt.show()
        x = 0



    def visualize_digraph_alp_layout(self, graph: GraphViewer):
        """
        In visualize function G is an object of. Creates and display a directed graph with a given list.
        :return:
        """
        pos = {'R1': (1, 0), 'A1': (1, 1), 'A2': (2, 3), 'A3': (3, 2), 'A4': (0.76, 1.80), 'A5': (0, 2)}
        graph.graph = nx.DiGraph()
        graph.graph.add_weighted_edges_from(graph.visual)
        positions = nx.get_node_attributes(graph.graph, 'R1')
        print("Node positions: ", positions)
        nx.set_node_attributes(graph.graph, positions)
        nx.draw_networkx(graph.graph)
        plt.show()

    def visualize_cost_evolution(self, iterations_list, instance_name):
        iterations_numbers = [key + 1 for key, index in enumerate(iterations_list)]
        plt.plot(iterations_numbers, iterations_list)
        plt.title(f"ACO - Evolução do custo por iteração\n ({instance_name})")
        plt.ylabel('Custo')
        plt.xlabel('Iterações')

        plt.show()
        pass
