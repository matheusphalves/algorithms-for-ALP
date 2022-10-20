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

class Runway:
    """
    Example of an Runaway (used by runaway list)
    runaway_name    airplane_index:landing_time
    Runway 1        1:125 5:201 4:56 –
    Runway 2        2:108 3:184 6:300 8:655
    Runway 3        7:54 10:407 9:520 –
    """
    def __init__(self, index, runway_name, solution_dict={}):
        """
        :param index: Index of Runaway
        :param runway_name: Just an alias for the Runaway
        :param solution_dict: List of selected Aircraft (with landing time assigned)
        """
        self.index = index
        self.runway_name = runway_name
        self.solution_dict = solution_dict
        self.runway_cost = 0


    def compute_landing_costs(self):
        """
        Compute the total cost associate to landing sequence choosen.
        :return:
        """
        runaway_cost = 0
        for key, aircraft in self.solution_dict.items():
            runaway_cost += aircraft.penality_cost_computed
        self.runway_cost = runaway_cost # avoid multiple sums
