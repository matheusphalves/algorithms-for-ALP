from algorithms_ALP.src.algorithms.ACO.ACOGraphViewer import ACOGraphViewer
from algorithms_ALP.src.algorithms.ACO.ACOReport import ACOReport
from algorithms_ALP.src.algorithms.ACO.ACOSolver import ACOSolver
from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler

def load_files(root_path, airland_range = [1,8]):
    df_data_list = []
    try:
        for airland in range(airland_range[0], airland_range[1]+1):
            if airland != 6:
                df = DataFrameHandler.read_csv_data(root_path + f'\\airland{str(airland)}.csv')
                df_data_list.append(df)
    except Exception as ex:
        print("Error!")
    return df_data_list

if __name__ == '__main__':
    #sample_data_path = 'C:\\Users\\Matheus Phelipe\\Desktop\workspace\\algorithms-for-ALP\\algorithms_ALP\\src\\sample_data\\or_library'
    sample_data_path = 'C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\src\\sample_data\\or_library'


    df_data_list = load_files(sample_data_path, [3,3])



    for df in df_data_list:
        for runaway_number in range(1, 2):
            print(f"Solving instance with {len(df)} planes with {runaway_number} runaways available.")
            aco_solver = ACOSolver(
                runaway_number = runaway_number,  # runaway_number: amount of runways available
                number_of_ants = 100,  # number_of_ants: amount of Ants to build solutions
                evaporation_rate = 0.9,  # evaporation_rate: rate at which pheromone evaporates
                pheromone_rate = 1.2,  # pheromone_intensity: constant added to the best path
                alpha = 5,  # alpha: weighting of pheromone
                beta = 1.6,  # beta: weighting of heuristic (visibility of ants)
                beta1 = 7,  # beta1: weighting of heuristic (priority)
                beta2 = 1)  # beta2: weighting of heuristic (cost penality)
            alp = ALPInstance(df)
            alp.build_ALP_instance()
            aco_solver.start(alp_instance=alp, max_iterations=100)
            aco_report = ACOReport(aco_solver.local_glorious_ant)
            aco_report.generate_report()



    # aco_graph = ACOGraphViewer()
    # aco_graph.visualize_best_solution(aco_solver.local_glorious_ant)