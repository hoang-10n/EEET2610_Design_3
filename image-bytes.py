import numpy as np
import os
a = np.array([1,2,3,4,5,6])
print("Original: ")
print(a)
a_bytes = a.tobytes()
a2 = np.frombuffer(a_bytes, dtype=a.dtype)
print("After: ")
print(a2)
print(np.array_equal(a, a2))