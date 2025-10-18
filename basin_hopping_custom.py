import numpy as np
from scipy.optimize import minimize
import os
from visualize import visualize_2d, generate_report, rastrigin

class BasinHopping:
    def __init__(self, func, bounds, niter=200, T=1.0, stepsize=0.5, 
                 interval=50, minimizer_method='L-BFGS-B'):
        self.func = func
        self.bounds = np.array(bounds)
        self.lower = self.bounds[:, 0]
        self.upper = self.bounds[:, 1]
        self.dim = len(bounds)
        self.niter = niter
        self.T = T
        self.stepsize = stepsize
        self.interval = interval
        self.minimizer_method = minimizer_method
        
        self.nfev = 0
        self.path = []
        self.energies = []
        self.accepted_count = 0
        self.rejected_count = 0
        self.distances_from_origin = []
        self.step_sizes = []
        self.current_stepsize = stepsize
        
    def take_step(self, x):
        x_new = x.copy()
        for i in range(self.dim):
            x_new[i] += np.random.uniform(-self.current_stepsize, self.current_stepsize)
        x_new = np.clip(x_new, self.lower, self.upper)
        return x_new
    
    def local_minimization(self, x):
        result = minimize(
            self.func, 
            x, 
            method=self.minimizer_method, 
            bounds=self.bounds
        )
        self.nfev += result.nfev
        return result.x, result.fun
    
    def accept_reject(self, f_new, f_old):
        if f_new < f_old:
            return True
        
        if self.T == 0:
            return False
        
        delta_f = f_new - f_old
        probability = np.exp(-delta_f / self.T)
        return np.random.random() < probability
    
    def adjust_stepsize(self, iteration):
        if iteration % self.interval == 0 and iteration > 0:
            total = self.accepted_count + self.rejected_count
            if total > 0:
                acceptance_rate = self.accepted_count / total
                
                if acceptance_rate > 0.6:
                    self.current_stepsize *= 1.1
                elif acceptance_rate < 0.4:
                    self.current_stepsize *= 0.9
                
                self.current_stepsize = np.clip(
                    self.current_stepsize, 
                    0.1 * self.stepsize, 
                    5.0 * self.stepsize
                )
    
    def optimize(self, x0):
        x0 = np.array(x0).flatten()
        
        x_current, f_current = self.local_minimization(x0)
        self.path.append(x_current.copy())
        self.energies.append(f_current)
        self.distances_from_origin.append(np.linalg.norm(x_current))
        
        x_best = x_current.copy()
        f_best = f_current
        
        for iteration in range(self.niter):
            self.step_sizes.append(self.current_stepsize)
            
            x_trial = self.take_step(x_current)
            
            x_new, f_new = self.local_minimization(x_trial)
            
            if self.accept_reject(f_new, f_current):
                self.accepted_count += 1
                x_current = x_new.copy()
                f_current = f_new
                self.path.append(x_current.copy())
                
                if f_current < f_best:
                    x_best = x_current.copy()
                    f_best = f_current
            else:
                self.rejected_count += 1
            
            self.energies.append(f_best)
            self.distances_from_origin.append(np.linalg.norm(x_best))
            
            self.adjust_stepsize(iteration)
        
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
            'best_values': np.array(self.energies),
            'distances': np.array(self.distances_from_origin),
            'step_sizes': np.array(self.step_sizes)
        }

def run_experiments(dimensions, use_scipy=False, output_folder='basin_hopping'):
    from scipy.optimize import basinhopping as scipy_basinhopping
    
    os.makedirs(output_folder, exist_ok=True)
    
    results_custom = {}
    results_scipy = {}
    
    for dim in dimensions:
        bounds = [(-5.12, 5.12)] * dim
        # x0 = np.random.uniform(-5.12, 5.12, dim)
        x0 = np.ones(dim) * 2.5
        
        optimizer = BasinHopping(
            rastrigin, 
            bounds, 
            niter=500,
            T=2.0,
            stepsize=1.5
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
            minimizer_kwargs = {'method': 'L-BFGS-B', 'bounds': bounds}
            result_scipy = scipy_basinhopping(
                rastrigin,
                x0,
                niter=500,
                T=2.0,
                stepsize=1.5,
                minimizer_kwargs=minimizer_kwargs
            )
            
            results_scipy[dim] = {
                'x0': x0.copy(),
                'x_best': result_scipy.x,
                'f_best': result_scipy.fun,
                'nfev': result_scipy.nfev
            }
    
    generate_report(results_custom, results_scipy, dimensions, use_scipy, output_folder, 'Basin Hopping')
    
    return results_custom, results_scipy

if __name__ == "__main__":
    dimensions = [2, 3, 4, 5, 6]
    use_scipy_comparison = False
    output_folder = 'basin_hopping'
    
    results_custom, results_scipy = run_experiments(
        dimensions=dimensions,
        use_scipy=use_scipy_comparison,
        output_folder=output_folder
    )
    
    visualize_2d(results_custom, output_folder, 'Basin Hopping')