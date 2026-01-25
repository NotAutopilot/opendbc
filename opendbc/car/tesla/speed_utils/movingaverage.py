"""Moving average utility for speed smoothing"""
import queue


class MovingAverage:
    def __init__(self, length: int):
        self.length = length
        self.reset()

    def reset(self):
        self.queue = queue.Queue(maxsize=self.length)
        self.sum = 0

    def add(self, sample: float) -> float:
        if self.queue.full():
            self.sum -= self.queue.get_nowait()
        self.queue.put_nowait(sample)
        self.sum += sample
        return self.sum / self.queue.qsize()

    def full(self) -> bool:
        return self.queue.full()
