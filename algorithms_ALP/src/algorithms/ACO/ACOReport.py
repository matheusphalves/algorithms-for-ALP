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
from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler
import datetime


class ACOReport:
    def __init__(self, ant: Ant):
        self.ant = ant
        self.report_dict = {}

    def save_runaway_report(self, runaway_dict, file_name, file_path=None):
        try:
            df = DataFrameHandler.dict_to_df(runaway_dict)
            DataFrameHandler.save_df_to_csv(df, file_name=file_name, file_path=file_path)
        except Exception as ex:
            print(f"Failed on save report... \n{str(ex)}")

    def generate_scheduled_times_report(self, cycle_number=0, custom_label=''):
        print(f"Generating report. {len(self.ant.runaways_dict)} runaways was detected in this solution.")

        for run_index, (runaway_index, runaway) in enumerate(self.ant.runaways_dict.items()):
            execution_date = datetime.datetime.now().strftime('%d_%m_%Y__%H_%M')
            runaway_dict = {}
            for air_index, (aircraft_index, aircraft) in enumerate(runaway.solution_dict.items()):
                runaway_dict[str(air_index + 1)] = {
                    'id': aircraft.index - 2,
                    # 'appearance_time': aircraft.appearance_time,
                    'Ei': aircraft.earliest_landing_time,
                    'xi': aircraft.landing_time,
                    'Tai': aircraft.target_landing_time,
                    'Li': aircraft.latest_landing_time,
                    # 'Ci-': aircraft.penality_cost_earliest,
                    # 'Ci+': aircraft.penality_cost_latest,
                    'COST': aircraft.penality_cost_computed
                }
            self.save_runaway_report(runaway_dict=runaway_dict, file_name=f'Cycle_{cycle_number}_{custom_label}_Planes_{len(runaway.solution_dict.items())}_R{run_index + 1}_{execution_date}')
        print(f"Report generated.")

    def generate_global_results_report(self):
        pass

    def generate_aco_settings_result_report(self):
        pass

    def generate_full_report_report(self):
        pass
