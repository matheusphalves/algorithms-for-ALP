#  """
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
#  """


class Aircraft:

    def __init__(self, index, data):
        """
        :param appearance_time:
        :param earliest_landing_time:
        :param target_landing_time:
        :param latest_landing_time:
        :param penality_cost_earliest:
        :param penality_cost_latest:
        """
        self.index = index
        self.appearance_time = data['appearance_time']
        self.earliest_landing_time = data['earliest_landing_time']
        self.target_landing_time = data['target_landing_time']
        self.latest_landing_time = data['latest_landing_time']
        self.penality_cost_earliest = data['penality_cost_earliest']
        self.penality_cost_latest = data['penality_cost_latest']
        self.landing_time = None
        self.penality_cost_computed = None
        self.probability_of_choose = 0

