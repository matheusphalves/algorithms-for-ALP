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

class Runaway:
    """
    Example of an Runaway (used by runaway list)
    runaway_name    airplane_index:landing_time
    Runway 1        1:125 5:201 4:56 –
    Runway 2        2:108 3:184 6:300 8:655
    Runway 3        7:54 10:407 9:520 –
    """
    def __init__(self, index, runaway_name, solution_list=[]):
        """
        :param index: Index of Runaway
        :param runaway_name: Just an alias for the Runaway
        :param solution_list: List of selected Aircraft (with landing time assigned)
        """
        self.index = index
        self.runaway_name = runaway_name
        self.solution_list = solution_list

