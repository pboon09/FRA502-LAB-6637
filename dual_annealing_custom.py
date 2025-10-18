import numpy as np
from scipy.optimize import minimize
import os
from visualize import visualize_2d, generate_report, rastrigin

class DualAnnealing:
    def __init__(self, func, bounds, initial_temp=5230.0, visit=2.62, accept=-5.0, 
                 restart_temp_ratio=2e-5, maxiter=1000, maxfun=1e7):
        self.func = func
        self.bounds = np.array(bounds)
        self.lower = self.bounds[:, 0]
        self.upper = self.bounds[:, 1]
        self.dim = len(bounds)
        self.initial_temp = initial_temp
        self.visit = visit
        self.accept = accept
        self.restart_temp_ratio = restart_temp_ratio
        self.maxiter = maxiter
        self.maxfun = int(maxfun)
        
        self.nfev = 0
        self.path = []
        self.temperatures = []
        self.best_values = []
        self.accepted_count = 0
        self.rejected_count = 0
        self.distances_from_origin = []
        
    def visiting_distribution(self, x, temperature):
        scale = temperature ** (1.0 / (self.dim - 1.0))
        factor1 = (self.visit - 1.0) * np.log(temperature)
        factor2 = np.log(self.visit - 1.0)
        factor3 = (4.0 - self.visit) * factor2
        sigmax = np.exp(factor1 + factor3) * scale
        
        u = np.random.uniform(-1, 1, size=self.dim)
        factor = sigmax * u / (1.0 - (1.0 - self.visit) * u**2)
        
        x_new = x + factor
        x_new = np.clip(x_new, self.lower, self.upper)
        return x_new
    
    def acceptance_probability(self, delta_e, temperature):
        if delta_e < 0:
            return 1.0
        beta = 1.0 / temperature
        qa = self.accept
        exponent = (1.0 - qa) * beta * delta_e
        if exponent < 1.0:
            p = (1.0 - exponent) ** (1.0 / (1.0 - qa))
            return min(1.0, p)
        return 0.0
    
    def temperature_schedule(self, step):
        return self.initial_temp * (step + 1) ** (-1.0 / (self.dim - 1.0))
    
    def local_search(self, x):
        result = minimize(self.func, x, method='L-BFGS-B', bounds=self.bounds)
        self.nfev += result.nfev
        return result.x, result.fun
    
    def optimize(self, x0=None):
        if x0 is None:
            x_current = np.random.uniform(self.lower, self.upper)
        else:
            x_current = np.array(x0).flatten()
        
        f_current = self.func(x_current)
        self.nfev += 1
        self.path.append(x_current.copy())
        self.best_values.append(f_current)
        self.distances_from_origin.append(np.linalg.norm(x_current))
        
        x_best = x_current.copy()
        f_best = f_current
        
        step = 0
        restart_temp = self.initial_temp * self.restart_temp_ratio
        
        while step < self.maxiter and self.nfev < self.maxfun:
            temperature = self.temperature_schedule(step)
            
            if temperature < restart_temp:
                temperature = self.initial_temp
            
            self.temperatures.append(temperature)
            
            x_new = self.visiting_distribution(x_current, temperature)
            f_new = self.func(x_new)
            self.nfev += 1
            
            delta_e = f_new - f_current
            p_accept = self.acceptance_probability(delta_e, temperature)
            
            if np.random.random() < p_accept:
                self.accepted_count += 1
                x_current = x_new.copy()
                f_current = f_new
                self.path.append(x_current.copy())
                
                if self.nfev < self.maxfun:
                    x_local, f_local = self.local_search(x_current)
                    if f_local < f_current:
                        x_current = x_local.copy()
                        f_current = f_local
                        self.path.append(x_current.copy())
                
                if f_current < f_best:
                    x_best = x_current.copy()
                    f_best = f_current
            else:
                self.rejected_count += 1
            
            self.best_values.append(f_best)
            self.distances_from_origin.append(np.linalg.norm(x_best))
            step += 1
        
        return x_best, f_best
    
    def get_statistics(self):
        total_moves = self.accepted_count + self.rejected_count
        acceptance_rate = self.accepted_count / total_moves if total_moves > 0 else 0
        
        return {
            'nfev': self.nfev,
            'path_length': len(self.path),
            'accepted_count': self.accepted_count,
            'rejected_count': self.rejected_count,
            'acceptance_rate': acceptance_rate,
            'temperatures': np.array(self.temperatures),
            'best_values': np.array(self.best_values),
            'distances': np.array(self.distances_from_origin)
        }

def run_experiments(dimensions, use_scipy=False, output_folder='dual_annealing'):
    from scipy.optimize import dual_annealing as scipy_dual_annealing
    
    os.makedirs(output_folder, exist_ok=True)
    
    results_custom = {}
    results_scipy = {}
    
    for dim in dimensions:
        bounds = [(-5.12, 5.12)] * dim
        # x0 = np.random.uniform(-5.12, 5.12, dim)
        x0 = np.ones(dim) * 2.5
        
        optimizer = DualAnnealing(
            rastrigin, 
            bounds, 
            initial_temp=5230.0,
            visit=2.62,
            accept=-5.0,
            maxiter=1000
        )
        
        x_best_custom, f_best_custom = optimizer.optimize(x0)
        stats = optimizer.get_statistics()
        
        results_custom[dim] = {
            'x0': x0.copy(),
            'x_best': x_best_custom,
            'f_best': f_best_custom,
            'stats': stats,
            'optimizer': optimizer
        }
        
        if use_scipy:
            result_scipy = scipy_dual_annealing(
                rastrigin,
                bounds,
                x0=x0,
                maxiter=1000,
                initial_temp=5230,
                visit=2.62,
                accept=-5.0
            )
            
            results_scipy[dim] = {
                'x0': x0.copy(),
                'x_best': result_scipy.x,
                'f_best': result_scipy.fun,
                'nfev': result_scipy.nfev
            }
    
    generate_report(results_custom, results_scipy, dimensions, use_scipy, output_folder, 'Dual Annealing')
    
    return results_custom, results_scipy

if __name__ == "__main__":
    dimensions = [2, 3, 4, 5, 6]
    use_scipy_comparison = False
    output_folder = 'dual_annealing'
    
    results_custom, results_scipy = run_experiments(
        dimensions=dimensions,
        use_scipy=use_scipy_comparison,
        output_folder=output_folder
    )
    
    visualize_2d(results_custom, output_folder, 'Dual Annealing')