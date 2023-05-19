import time
from collections import defaultdict

class Timer:
    def __init__(self):
        self.dict = defaultdict(float)
        self.reset()

    def reset(self):
        self.t = time.perf_counter()

    def __call__(self, key):
        t = time.perf_counter()
        self.dict[key] += t - self.t
        self.t = t
        return self.dict[key]

    def __str__(self):
        return str(dict(self.dict))
