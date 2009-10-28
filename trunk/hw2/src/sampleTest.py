import statutil
import random

class TestClass:
    def __init__(self, value, weight):
        self.value = value
        self.weight = weight

test1 = [TestClass(i, random.random()) for i in range(10)]
for t in test1:
    print t.value, " ", t.weight
sampled = statutil.lowVarianceSample(test1, 20)
for s in sampled:
    print s.value, " ", s.weight
