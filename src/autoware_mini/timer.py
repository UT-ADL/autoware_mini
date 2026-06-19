import time

class Timer:
    def __init__(self, name=None):
        self.name = name
        self.total_time = 0.0
        self.call_count = 0
        self.start_time = None

    def __enter__(self):
        self.start_time = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        elapsed = time.perf_counter() - self.start_time
        self.total_time += elapsed
        self.call_count += 1
        self.start_time = None
        print(f"{self.name} - call time: {elapsed:.6f}s, avg time: {self.total_time / self.call_count:.6f}s")

    def __call__(self, func):
        def wrapper(*args, **kwargs):
            with self:
                return func(*args, **kwargs)
        return wrapper

def time_callback(func):
    return Timer(func.__qualname__)(func)
