from scipy.optimize import dual_annealing
import numpy as np

def rastrigin(x):
    A = 10
    n = len(x)
    return A * n + np.sum(x**2 - A * np.cos(2 * np.pi * x))

dimension = 2
bounds = [(-5.12, 5.12)] * dimension
x0 = np.random.uniform(-5.12, 5.12, dimension)

print(f"Initial point: {x0}")
print(f"Initial value: {rastrigin(x0):.6f}\n")

result = dual_annealing(
    rastrigin,              # Objective function to minimize
    bounds,                 # Bounds for each dimension
    x0=x0,                  # Starting point (optional)
    maxiter=1000,           # Maximum number of global search iterations
    initial_temp=5230,      # Initial temperature for annealing
    visit=2.62,             # Visiting parameter qv (1, 3] - controls jump distance
    accept=-5.0             # Acceptance parameter qa - controls acceptance probability
)

print(f"Global minimum: x = {result.x}")
print(f"Function value: f(x) = {result.fun:.10f}")
print(f"Number of iterations: {result.nit}")
print(f"Number of function evaluations: {result.nfev}")
print(f"Success: {result.success}")
print(f"Message: {result.message}")