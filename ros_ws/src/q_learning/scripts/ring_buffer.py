import random


class RingBuffer(object):
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0

    def push(self, item):
        if not self.full():
            self.buffer.append(None)
        self.buffer[self.position] = item
        self.position = (self.position + 1) % self.capacity

    def sample(self, batch_size):
        return random.sample(self.buffer, batch_size)

    def full(self):
        return len(self.buffer) >= self.capacity

    def size(self):
        return len(self.buffer)
