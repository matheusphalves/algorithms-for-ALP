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
from algorithms_ALP.src.algorithms.ACO.parallel.ACOParallelSolver import ACOParallelSolver
from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler


def load_files(root_path, airland_range = [2,2]):
    df_data_list = []
    try:
        for airland in range(airland_range[0], airland_range[1]+1):
            df = DataFrameHandler.read_csv_data(root_path + f'\\airland{str(airland)}.csv')
            df_data_list.append(df)
    except Exception as ex:
        print("Error!")
    return df_data_list

if __name__ == '__main__':
    # sample_data_path = 'C:\\Users\\Matheus Phelipe\\Desktop\\workspace\\algorithms-for-ALP\\algorithms_ALP\\src\\sample_data\\or_library'
    sample_data_path = 'C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\src\\sample_data\\or_library'


    df_data_list = load_files(sample_data_path, [8, 8])


    counter = 1
    instance_counter = 1
    for df in df_data_list:
        for runaway_number in range(1, 2):
            print(f"Solving instance with {len(df)} planes with {runaway_number} runaways available.")
            aco_solver = ACOParallelSolver(
            # CICLO 1
            # runaway_number = runaway_number,  # runaway_number: amount of runways available
            # number_of_ants = 50,  # number_of_ants: amount of Ants to build solutions
            # evaporation_rate = 0.9,  # evaporation_rate: rate at which pheromone evaporates
            # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
            # alpha = 1,  # alpha: weighting of pheromone
            # beta = 0.5,  # beta: weighting of heuristic (visibility of ants)
            # beta1 = 2,  # beta1: weighting of heuristic (priority)
            # beta2 = 8)  # beta2: weighting of heuristic (cost penality)
            # #CICLO 3
            runaway_number=runaway_number,  # runaway_number: amount of runways available
            number_of_ants=int(2*len(df)),  # number_of_ants: amount of Ants to build solutions
            evaporation_rate=0.1,  # evaporation_rate: rate at which pheromone evaporates
            pheromone_rate=10,  # pheromone_intensity: constant added to the best path
            alpha=3,  # alpha: weighting of pheromone
            beta=1,  # beta: weighting of heuristic (visibility of ants)
            beta1=2,  # beta1: weighting of heuristic (priority)
            beta2=0.1)  # beta2: weighting of heuristic (cost penality)
            alp = ALPInstance(df)
            alp.build_ALP_instance()
            aco_solver.start(alp_instance=alp, max_iterations=200)
            counter +=1
        instance_counter += 1