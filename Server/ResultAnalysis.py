import numpy as np
import matplotlib.pyplot as plt

data = []
with open('Res.txt', 'r') as f:
    data = [item.strip('\n').split('\t') for item in f.readlines()]

t = [] 
p = [] 

for frame in data:
    t.append(float(frame[0]))
    p.append(float(frame[1]))

print(p)

plt.plot(t, p)
plt.grid(True)
    
plt.show()
