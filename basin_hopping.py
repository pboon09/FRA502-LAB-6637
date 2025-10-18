from scipy.optimize import basinhopping
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

minimizer_kwargs = {
    'method': 'L-BFGS-B',   # Local minimization method
    'bounds': bounds        # Bounds for local minimizer
}

result = basinhopping(
    rastrigin,              # Objective function to minimize
    x0,                     # Starting point (required)
    niter=500,              # Number of basin hopping iterations
    T=2.0,                  # Temperature parameter for acceptance test
    stepsize=1.5,           # Maximum step size for random displacement
    minimizer_kwargs=minimizer_kwargs  # Options for local minimizer
)

print(f"Global minimum: x = {result.x}")
print(f"Function value: f(x) = {result.fun:.10f}")
print(f"Number of iterations: {result.nit}")
print(f"Number of function evaluations: {result.nfev}")
print(f"Number of local minimizations: {result.nfev // result.nit if result.nit > 0 else 0}")
print(f"Success: {result.lowest_optimization_result.success}")
print(f"Message: {result.message}")