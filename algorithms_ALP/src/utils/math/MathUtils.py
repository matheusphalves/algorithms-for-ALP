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

import numpy as np
import random


class MathUtils:
    """
    Provides basic Math operations for ALP algorithms.
    """
    def __init__(self):
        pass

    @staticmethod
    def generate_random_square_matrix(size: int, min_dist: int, max_dist: int):
        stuffs = np.zeros((size, size), dtype=int)
        for i in range(size):
            for j in range(size):
                if j > i:
                    stuffs[i, j] = random.randint(min_dist, max_dist)
                elif j < i:
                    stuffs[i, j] = stuffs[j, i]
        return stuffs

    @staticmethod
    def generate_square_matrix(size: int, distance_matrix: list):
        stuffs = np.zeros((size, size), dtype=int)
        for i in range(size):
            for j in range(size):
                if j > i:
                    stuffs[i, j] = distance_matrix[i][j]
                elif j < i:
                    stuffs[i, j] = stuffs[j, i]
        return stuffs

    @staticmethod
    def matrix_list_to_np_array(matrix: list, dtype=float):
        return np.array(matrix, dtype=dtype)

    @staticmethod
    def join_lists_to_np_array(data_list: list):
        return np.array([data for data in data_list])

    @staticmethod
    def join_lists(list_a, list_b):
        return list_a + list_b