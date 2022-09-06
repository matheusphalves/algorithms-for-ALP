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

from algorithms_ALP.src.utils.handlers.DataFrameHandler import DataFrameHandler
from algorithms_ALP.src.utils.handlers.FileHandler import FileHandler
from algorithms_ALP.src.utils.parsers.AbstractParser.AbstractParser import AbstractParser

import re


class ALPParser(AbstractParser):

    def __init__(self) -> None:
        super().__init__()
        self.planes_number = None
        self.freeze_time = None

    def parse_content(self, file_path):
        content = FileHandler.read_file(file_path, read_mode='r')
        alp_parser = ALPParser()
        status, parsed_content = alp_parser.parser(content)
        if status:
            df = DataFrameHandler.dict_to_df(parsed_content)
            DataFrameHandler.save_df_to_csv(df, 'airland')

    def parser(self, content_str):
        try:
            self.planes_number, self.freeze_time = self.get_planes_number_and_freeze_time(content_str)
            if self.planes_number is not None:
                data = self.get_data_from_all_planes(content_str)
                if data:
                    return True, data
                else:
                    return False, ['Failed to get data from files']
            else:
                return False, ['Failed to get number of planes']

        except Exception as ex:
            raise BaseException('Failed to parse file:\n' + str(ex))
        pass

    def get_planes_number_and_freeze_time(self, content_str):
        """
        number of planes (p), freeze time
        :param content_str:
        :return: list
        """
        try:
            search = re.search("([0-9]+\s[0-9]+)", content_str)
            return search[0].split(' ')
        except Exception as ex:
            return None

    def get_data_from_all_planes(self, content_str):
        """
        for each plane i (i=1,...,p):
           appearance time, earliest landing time, target landing time,
           latest landing time, penalty cost per unit of time for landing
           before target, penalty cost per unit of time for landing
           after target
        :return: dict
        """
        content_split = content_str.split('\n')[1:]
        planes = {}
        plane_counter = 0
        for row_content in content_split:

            times = self.get_time_sequence(row_content)

            if len(times) == 6:
                planes[str(plane_counter)] = {
                    'appearance_time': times[0].strip(),
                    'earliest_landing_time': times[1].strip(),
                    'target_landing_time': times[2].strip(),
                    'latest_landing_time': times[3].strip(),
                    'penality_cost_earliest': times[4].strip(),
                    'penality_cost_latest': times[5].strip(),
                    'separation_times': self.get_separation_time_from_plane(content_str.split(row_content)[1:][0])
                }
                plane_counter +=1

        return planes

    def get_time_sequence(self, row_content, separator=' '):
        return [time_item.strip() for time_item in row_content.split(separator) if len(time_item.strip())]

    def get_separation_time_from_plane(self, content_str):
        """
        for each plane j (j=1,...p): separation time required after
                                    i lands before j can land
        :return: list
        """
        plane_counter = 0
        separation_list = []
        if int(self.planes_number) > 0:
            content_split = content_str.split('\n')
            for content_row in content_split:
                list_size = len(separation_list)
                times_row = self.get_time_sequence(content_row)
                if list_size <= int(self.planes_number) and list_size + len(times_row) <= int(self.planes_number):
                    plane_counter += len(times_row)
                    [separation_list.append(single_time) for single_time in times_row]
                else:
                    break
        return separation_list
