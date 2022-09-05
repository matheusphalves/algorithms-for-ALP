from algorithms_ALP.src.ALPParser import ALPParser
from algorithms_ALP.src.algorithms.ACO.ACOSolver import ACOSolver
from algorithms_ALP.src.algorithms.ACO.ALPInstance import ALPInstance
from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler
from algorithms_ALP.src.utils.handlers.FileHandler import FileHandler

if __name__ == '__main__':
    # df = DataFrameHandler.read_csv_data('C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland_1662239820256920500.csv')
    df = DataFrameHandler.read_csv_data('C:\\Users\\mathe\\Desktop\\workspace\\algorithms-aircraft-landing-problems\\algorithms_ALP\\tmp\\airland8_1662311765763438000.csv')
    alp = ALPInstance(df)
    alp.build_ALP_instance()

    aco_solver = ACOSolver(
        runaway_number = 1,
        number_of_ants = 100,
        evaporation_rate = 0.2,
        pheromone_intensity = 1,
        beta_evaporation_rate = 1)
    aco_solver.start(alp_intance=alp)
    x = 0

    # alp_parser = ALPParser()
    # alp_parser.parse_content('D:\\testing\\airland1.txt')
