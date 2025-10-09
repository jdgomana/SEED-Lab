'''
This program utilizes numerous list functions to gather a variety of information from a data file.
'''

import numpy as np

with open('datafile.txt', 'r') as f:
    b = eval(f.read())

# Max
print("Max: ", max(b))

# Min
print("Min: ", min(b))

# Index of 38
print("Index of 38: ", b.index(38))

# Number(s) repeated most and # of times repeated
timesRepeated = max([b.count(x) for x in b])
mostRepeated = set([x for x in b if b.count(x) == timesRepeated])
print("Most repeated number(s): ", mostRepeated)
print("Number of times repeated: ", timesRepeated)

# Sorted list
array = np.array(b)
sortedArray = np.sort(array)
print("Sorted list: ", sortedArray)

# Sorted even numbers
evenNumbers = [x for x in b if x % 2 == 0]
sortedEven = set(np.sort(np.array(evenNumbers)))
print("Sorted set of evens: ", sortedEven)
