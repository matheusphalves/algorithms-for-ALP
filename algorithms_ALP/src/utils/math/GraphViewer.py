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
import networkx as nx
import matplotlib.pyplot as plt


class GraphViewer:
    def __init__(self):
        """
        visual is a list which stores all the set of edges that constitutes a graph
        """
        self.visual = []

    def add_weigthed_edge(self, a, b, weight=None):
        """
        addEdge function inputs the vertices of an edge and appends it to the visual list
        :param a:
        :param b:
        :param weight:
        :return:
        """
        temp = [a, b, weight]
        self.visual.append(temp)

    def visualize_digraph(self):
        """
        In visualize function G is an object of. Creates and display a directed graph with a given list.
        :return:
        """
        graph = nx.DiGraph()
        graph.add_weighted_edges_from(self.visual)
        nx.draw_networkx(graph)
        plt.title("ACO - Path Direction")
        plt.show()

# Example of usage

#G = GraphViewer()

#G.add_weigthed_edge('R1','A1', 0.9)
#G.add_weigthed_edge('R1', 'A2', 0.1)
#G.visualize_digraph()
