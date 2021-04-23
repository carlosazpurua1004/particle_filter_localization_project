#!/usr/bin/env python3

from particle_filter import draw_random_sample

if __name__ == "__main__":
    sampling_set = [1,2,3,4,5,6]
    samples = draw_random_sample([1,2,3,4,5,6], [.1, .3, 0.05, 0.25, 0.1, 0.2], 1000)
    print([samples.count(num) for num in sampling_set])