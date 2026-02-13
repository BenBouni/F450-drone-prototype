import numpy as np
import matplotlib.pyplot as plt

# Sample data
x = np.linspace(0, 10, 100)
y = np.sin(x)

# Create a simple line graph
plt.figure(figsize=(10, 6))
plt.plot(x, y, label='sin(x)')
plt.xlabel('X axis')
plt.ylabel('Y axis')
plt.title('Simple Graph')
plt.legend()
plt.grid(True)
plt.show()