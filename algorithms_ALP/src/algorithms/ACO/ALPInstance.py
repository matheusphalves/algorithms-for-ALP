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

from algorithms_ALP.src.utils.math.MathUtils import MathUtils
import ast

class ALPInstance:
    """
    Build an instance of ALP to be solved by optimization algorithm.
    """
    def __init__(self, df_data):
        self.df_data = df_data
        self.separation_times_matrix = None
        self.aircraft_times = {}

    def build_ALP_instance(self):
        separation_times_list = []

        for index, aircraft_row in self.df_data.iterrows():
            self.aircraft_times[f'{str(index)}'] = {
                'appearance_time': aircraft_row['appearance_time'],
                'earliest_landing_time': aircraft_row['earliest_landing_time'],
                'target_landing_time': aircraft_row['target_landing_time'],
                'latest_landing_time': aircraft_row['latest_landing_time'],
                'penality_cost_earliest': aircraft_row['penality_cost_latest'],
                'penality_cost_latest': aircraft_row['penality_cost_latest']
            }
            list_aux = [float(time) for time in ast.literal_eval(aircraft_row['separation_times'])]
            separation_times_list.append(list_aux)

        self.separation_times_matrix = MathUtils.matrix_list_to_np_array(separation_times_list)