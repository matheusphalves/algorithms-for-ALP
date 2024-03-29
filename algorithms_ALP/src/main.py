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

#  MIT License
#
#
#  Permission is hereby granted, free of charge, to any person obtaining a copy
#  of this software and associated documentation files (the "Software"), to deal
#  in the Software without restriction, including without limitation the rights
#  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#  copies of the Software, and to permit persons to whom the Software is
#  furnished to do so, subject to the following conditions:
#
#
#from algorithms_ALP.src.algorithms.ACO.ACOGraphViewer import ACOGraphViewer
from algorithms_ALP.src.algorithms.ACO.ACOGraphViewer import ACOGraphViewer
from algorithms_ALP.src.algorithms.ACO.ACOReport import ACOReport
from algorithms_ALP.src.algorithms.ACO.ACOSolver import ACOSolver
from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler

def load_files(root_path, airland_range = [1,1]):
    df_data_list = []
    try:
        for airland in range(airland_range[0], airland_range[1]+1):
            df = DataFrameHandler.read_csv_data(root_path + f'\\airland{str(airland)}.csv')
            df_data_list.append({'airland': airland, 'df': df})
    except Exception as ex:
        print("Error!")
    return df_data_list

def load_files_new(root_path, airlands_info = [1, 1]):
    df_data_list = []
    try:
        for airland in airlands_info:
            df = DataFrameHandler.read_csv_data(root_path + f'\\airland{str(airland[0])}.csv')
            df_data_list.append({'airland': airland[0], 'runway': airland[1], 'df': df})
    except Exception as ex:
        print("Error!")
    return df_data_list

if __name__ == '__main__':
    # sample_data_path = 'C:\\Users\\Matheus Phelipe\\Desktop\\workspace\\algorithms-for-ALP\\algorithms_ALP\\src\\sample_data\\or_library'
    sample_data_path = 'C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\src\\sample_data\\or_library'


    # df_data_list = load_files_new(sample_data_path, [(1, 3), (2, 3), (3, 3), (4, 4), (5, 4), (8, 3)])
    # df_data_list = load_files_new(sample_data_path, [(1, 3), (2, 3), (3, 3), (4, 4), (5, 4)])
    df_data_list = load_files_new(sample_data_path, [(3, 3), (4, 4), (5, 4)])
    # df_data_list = load_files_new(sample_data_path, [(8, 2)])


    counter = 1
    instance_counter = 1
    for df in df_data_list:
        for runaway_number in range(1, df['runway'] + 1):
            print(f"Solving instance with {len(df['df'])} planes with {runaway_number} runaways available.")
            aco_solver = ACOSolver(
                # runaway_number=runaway_number,                              # runaway_number: amount of runways available
                # number_of_ants=int(len(df)*1.2),                                         # number_of_ants: amount of Ants to build solutions
                # evaporation_rate=0.3,                                       # evaporation_rate: rate at which pheromone evaporates
                # pheromone_rate=1.25,                                         # pheromone_intensity: constant added to the best path
                # alpha=2,                                                    # alpha: weighting of pheromone
                # beta=1.8,                                                   # beta: weighting of heuristic (visibility of ants)
                # beta1=4,                                                    # beta1: weighting of heuristic (priority)
                # beta2=2)                                                    # beta2: weighting of heuristic (cost penality)
                # runaway_number = runaway_number,  # runaway_number: amount of runways available
                # number_of_ants = 50,  # number_of_ants: amount of Ants to build solutions
                # evaporation_rate = 0.9,  # evaporation_rate: rate at which pheromone evaporates
                # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
                # alpha = 2,  # alpha: weighting of pheromone
                # beta = 0.5,  # beta: weighting of heuristic (visibility of ants)
                # beta1 = 4,  # beta1: weighting of heuristic (priority)
                # beta2 = 2)  # beta2: weighting of heuristic (cost penality)
            # # CICLO 1 FINAL PFV
            # runaway_number = runaway_number,  # runaway_number: amount of runways available
            # number_of_ants = 75,  # number_of_ants: amount of Ants to build solutions
            # evaporation_rate = 0.5,  # evaporation_rate: rate at which pheromone evaporates
            # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
            # alpha = 2,  # alpha: weighting of pheromone
            # beta = 1,  # beta: weighting of heuristic (visibility of ants)
            # beta1 = 6,  # beta1: weighting of heuristic (priority)
            # beta2 = 5)  # beta2: weighting of heuristic (cost penality)
            # # CICLO 2 FINAL PFV
            # runaway_number = runaway_number,  # runaway_number: amount of runways available
            # number_of_ants = 50,  # number_of_ants: amount of Ants to build solutions
            # evaporation_rate = 0.9,  # evaporation_rate: rate at which pheromone evaporates
            # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
            # alpha = 1,  # alpha: weighting of pheromone
            # beta = 0.5,  # beta: weighting of heuristic (visibility of ants)
            # beta1 = 2,  # beta1: weighting of heuristic (priority)
            # beta2 = 4)  # beta2: weighting of heuristic (cost penality)
            # #CICLO 3 FINAL PFV
            runaway_number=runaway_number,  # runaway_number: amount of runways available
            number_of_ants=len(df['df']),  # number_of_ants: amount of Ants to build solutions
            evaporation_rate=0.1,  # evaporation_rate: rate at which pheromone evaporates
            pheromone_rate=10,  # pheromone_intensity: constant added to the best path
            alpha=3,  # alpha: weighting of pheromone
            beta=1,  # beta: weighting of heuristic (visibility of ants)
            beta1=2,  # beta1: weighting of heuristic (priority)
            beta2=0.5)  # beta2: weighting of heuristic (cost penality)
            alp = ALPInstance(df['df'])
            alp.build_ALP_instance()
            aco_solver.start(alp_instance=alp, max_iterations=400)
            #aco_report = ACOReport(aco_solver.global_glorious_ant)
            #aco_report.generate_scheduled_times_report(cycle_number=3, custom_label=f"Airland {df['airland']} -  {runaway_number} pistas")
            aco_graph = ACOGraphViewer()
            aco_graph.visualize_cost_evolution(aco_solver.iterations_costs, instance_name=f"Ciclo 3 - Airland {df['airland']} - {runaway_number} pistas")
            #aco_graph.visualize_best_solution(aco_solver.global_glorious_ant, instance_name= "Airland {df['airland']}")
            # aco_report.generate_scheduled_times_report(cycle_number=0, custom_label=f"air{counter}_best_{aco_solver.global_glorious_ant.solution_cost}_time{aco_solver.time_execution.__str__().replace(':', '-')}_")
            counter +=1
        instance_counter += 1


    # CICLO 1 - 50 (CICLO 1) iterações Q0 = 0.9
    # runaway_number = runaway_number,  # runaway_number: amount of runways available
    # number_of_ants = 50,  # number_of_ants: amount of Ants to build solutions
    # evaporation_rate = 0.9,  # evaporation_rate: rate at which pheromone evaporates
    # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
    # alpha = 2,  # alpha: weighting of pheromone
    # beta = 0.5,  # beta: weighting of heuristic (visibility of ants)
    # beta1 = 2,  # beta1: weighting of heuristic (priority)
    # beta2 = 1)  # beta2: weighting of heuristic (cost penality)

    # CICLO 2 - 100 (CICLO 2) iterações Q0 = 0.9
    # runaway_number = runaway_number,  # runaway_number: amount of runways available
    # number_of_ants = 100,  # number_of_ants: amount of Ants to build solutions
    # evaporation_rate = 0.5,  # evaporation_rate: rate at which pheromone evaporates
    # pheromone_rate = 1,  # pheromone_intensity: constant added to the best path
    # alpha = 1,  # alpha: weighting of pheromone
    # beta = 1,  # beta: weighting of heuristic (visibility of ants)
    # beta1 = 5,  # beta1: weighting of heuristic (priority)
    # beta2 = 4)  # beta2: weighting of heuristic (cost penality)

    # CICLO 3.1 - 200 ITERAÇÕES Q0 = 0.9
    # runaway_number = runaway_number,  # runaway_number: amount of runways available
    # number_of_ants = 150,  # number_of_ants: amount of Ants to build solutions
    # evaporation_rate = 0.3,  # evaporation_rate: rate at which pheromone evaporates
    # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
    # alpha = 1,  # alpha: weighting of pheromone
    # beta = 1,  # beta: weighting of heuristic (visibility of ants)
    # beta1 = 4,  # beta1: weighting of heuristic (priority)
    # beta2 = 2)  # beta2: weighting of heuristic (cost penality)
    # alp = ALPInstance(df)
    # alp.build_ALP_instance()
    # aco_solver.start(alp_instance=alp, max_iterations=200)

    # CICLO 3.2 (CICLO 3) - 200 ITERAÇÕES Q0 = 0.3
    # runaway_number = runaway_number,  # runaway_number: amount of runways available
    # number_of_ants = (int(len(df) * 2)),  # number_of_ants: amount of Ants to build solutions
    # evaporation_rate = 0.1,  # evaporation_rate: rate at which pheromone evaporates
    # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
    # alpha = 3,  # alpha: weighting of pheromone
    # beta = 1,  # beta: weighting of heuristic (visibility of ants)
    # beta1 = 2,  # beta1: weighting of heuristic (priority)
    # beta2 = 0.1)

    # CICLO 3.2 - 200 ITERAÇÕES Q0 = 0.5
    # runaway_number = runaway_number,  # runaway_number: amount of runways available
    # number_of_ants = (int(len(df) * 2)),  # number_of_ants: amount of Ants to build solutions
    # evaporation_rate = 0.1,  # evaporation_rate: rate at which pheromone evaporates
    # pheromone_rate = 10,  # pheromone_intensity: constant added to the best path
    # alpha = 3,  # alpha: weighting of pheromone
    # beta = 1,  # beta: weighting of heuristic (visibility of ants)
    # beta1 = 2,  # beta1: weighting of heuristic (priority)
    # beta2 = 0.1)


    # aco_graph = ACOGraphViewer()
    # aco_graph.visualize_best_solution(aco_solver.local_glorious_ant)

    # runaway_number = runaway_number,  # runaway_number: amount of runways available
    # number_of_ants = 200,  # number_of_ants: amount of Ants to build solutions
    # evaporation_rate = 0.5,  # evaporation_rate: rate at which pheromone evaporates
    # pheromone_rate = 1.2,  # pheromone_intensity: constant added to the best path
    # alpha = 2,  # alpha: weighting of pheromone
    # beta = 1.5,  # beta: weighting of heuristic (visibility of ants)
    # beta1 = 6,  # beta1: weighting of heuristic (priority)
    # beta2 = 0.25)  # beta2: weighting of heuristic (cost penality)