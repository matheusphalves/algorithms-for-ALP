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

import pandas as pd
import os
import time


class DataFrameHandler:
    def __init__(self):
        pass

    @staticmethod
    def read_csv_data(file_path, usecols=None, sep=';'):
        try:
            data = pd.read_csv(file_path, sep=sep, skipinitialspace=True, usecols=usecols)
            return data
        except Exception as ex:
            raise BaseException(f'Failed to read data\n {str(ex)}')

    @staticmethod
    def dict_to_df(data_dict, orient='index'):
        return pd.DataFrame.from_dict(data_dict, orient=orient)

    @staticmethod
    def save_df_to_csv(data_frame, file_name, file_path=None):

        if file_path is None:
            file_path = os.getcwd() + '\\tmp\\'
            try:
                os.mkdir(file_path)
            except:
                pass

            file_path = os.path.join(file_path, f'{file_name}_{time.time_ns()}.csv')

        if data_frame.empty is False:
            data_frame.to_csv(file_path, sep=';')
            return True

        return False
