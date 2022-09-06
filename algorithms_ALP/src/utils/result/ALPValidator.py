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
from algorithms_ALP.src.utils.result.ALPResult import ALPResult


class ALPValidator:
    """
    Given a solution, evaluate whether it meets the expected criterios.
    """


    def __init__(self):
        pass

    @staticmethod
    def check_solution(self, alp_instance: ALPInstance, result: ALPResult):
        """
        Given a solution, evaluate whether it meets the expected criterios.
        :return: boolean
        """
        pass

    @staticmethod
    def is_landing_time_inside_landing_window():
        """
        The landing time must be within the landing widow [ei, li]
        :param self:
        :return: boolean
        """
        pass

    @staticmethod
    def is_interval_security_respected():
        """
        The interval of security must be respected (also called as separation time)
        :param self:
        :return: boolean
        """

        pass