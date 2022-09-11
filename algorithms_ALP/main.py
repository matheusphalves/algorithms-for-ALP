from algorithms_ALP.src.algorithms.ACO.ACOSolver import ACOSolver
from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler

if __name__ == '__main__':
    #df = DataFrameHandler.read_csv_data('C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland_1662239820256920500.csv')
    #df = DataFrameHandler.read_csv_data('C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland9_1662311765839387800.csv')
    #df = DataFrameHandler.read_csv_data('C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland_1662239820256920500.csv')
    df = DataFrameHandler.read_csv_data('C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland2_1662311765618525300.csv')
    alp = ALPInstance(df)
    alp.build_ALP_instance()

    aco_solver = ACOSolver(
        runaway_number=1,                                       # runaway_number: amount of runways available
        number_of_ants=100,                                     # number_of_ants: amount of Ants to build solutions
        evaporation_rate=0.9,                                  # evaporation_rate: rate at which pheromone evaporates
        pheromone_rate=1.2,                                      # pheromone_intensity: constant added to the best path
        alpha=3,                                                # alpha: weighting of pheromone
        beta=1.6,                                                 # beta: weighting of heuristic (visibility of ants)
        beta1=7,                                                # beta1: weighting of heuristic (priority)
        beta2=0.4)                                                # beta2: weighting of heuristic (cost penality)
    aco_solver.start(alp_intance=alp, max_iterations=100)
    x = 0

    # alp_parser = ALPParser()
    # alp_parser.parse_content('D:\\testing\\airland1.txt')

{
    "ALP_INPUT_PATH": "C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland_1662239820256920500.csv",
    "RUNAWAY_NUMBER": 1,
    "NUMBER_OF_ANTS": 100,
    "EVAPORATION_RATE": 0.9,
    "PHEROMONE_RATE": 1.2,
    "ALPHA": 3,
    "BETA": 1.6,
    "BETA1": 7,
    "BETA2": 0.5

}