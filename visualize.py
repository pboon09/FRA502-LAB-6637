import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

def rastrigin(x):
    A = 10
    n = len(x)
    return A * n + np.sum(x**2 - A * np.cos(2 * np.pi * x))

def visualize_2d(results_custom, output_folder, algorithm_name):
    dim = 2
    result = results_custom[dim]
    optimizer = result['optimizer']
    x0_2d = result['x0']
    x_best_2d = result['x_best']
    f_best_2d = result['f_best']
    path_2d = np.array(optimizer.path)
    stats = result['stats']
    
    N = 100
    x1 = np.linspace(-5.12, 5.12, N)
    x2 = np.linspace(-5.12, 5.12, N)
    X, Y = np.meshgrid(x1, x2)
    
    val = np.zeros((N, N))
    for i in range(N):
        for j in range(N):
            xx = np.array([X[i, j], Y[i, j]])
            val[i, j] = rastrigin(xx)
    
    fig1 = plt.figure(figsize=(14, 10))
    ax1 = fig1.add_subplot(1, 1, 1)
    
    CF = ax1.contourf(x1, x2, val, levels=40, cmap='viridis', alpha=0.85)
    CS = ax1.contour(x1, x2, val, levels=20, colors='black', linewidths=0.5, alpha=0.3)
    
    cbar = plt.colorbar(CF, ax=ax1)
    cbar.set_label('Function Value', rotation=270, labelpad=25, fontsize=14, weight='bold')
    cbar.ax.tick_params(labelsize=12)
    
    ax1.plot(x0_2d[0], x0_2d[1], 'r*', markersize=25, markeredgewidth=3, 
            markeredgecolor='darkred', label='Start', zorder=12)
    
    ax1.plot(path_2d[:, 0], path_2d[:, 1], 'cyan', alpha=0.7, linewidth=3.5, 
            label='Optimization Path', zorder=5)
    
    step_interval = max(1, len(path_2d) // 20)
    for i in range(0, len(path_2d), step_interval):
        ax1.plot(path_2d[i, 0], path_2d[i, 1], 'wo', markersize=12, 
                markeredgecolor='blue', markeredgewidth=2.5, zorder=9)
        ax1.text(path_2d[i, 0], path_2d[i, 1], str(i), fontsize=10, color='black', 
                ha='center', va='center', weight='bold', zorder=10,
                bbox=dict(boxstyle='circle,pad=0.15', facecolor='yellow', 
                         edgecolor='blue', alpha=0.9, linewidth=2))
    
    ax1.plot(x_best_2d[0], x_best_2d[1], 'g^', markersize=22, markeredgewidth=3, 
            markeredgecolor='darkgreen', label=f'Found Minimum', zorder=13)
    
    ax1.plot(0, 0, 'mo', markersize=18, markeredgewidth=3, markeredgecolor='purple', 
            label='True Global Minimum', zorder=13)
    
    info_text = f"STATISTICS\n"
    info_text += f"{'='*35}\n"
    info_text += f"Dimension: 2D\n"
    info_text += f"Start: ({x0_2d[0]:.3f}, {x0_2d[1]:.3f})\n"
    info_text += f"Found: ({x_best_2d[0]:.6f}, {x_best_2d[1]:.6f})\n"
    info_text += f"f(x) = {f_best_2d:.2e}\n"
    info_text += f"Function Evaluations: {stats['nfev']}\n"
    info_text += f"Path Steps: {stats['path_length']}\n"
    info_text += f"Accepted Moves: {stats['accepted_count']}\n"
    info_text += f"Rejected Moves: {stats['rejected_count']}\n"
    info_text += f"Acceptance Rate: {stats['acceptance_rate']:.2%}"
    
    ax1.text(0.02, 0.98, info_text, transform=ax1.transAxes, fontsize=11,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.85),
            family='monospace')
    
    ax1.set_xlabel('x₁', fontsize=16, weight='bold')
    ax1.set_ylabel('x₂', fontsize=16, weight='bold')
    ax1.set_title(f'{algorithm_name} - Optimization Path on Rastrigin Function', 
                 fontsize=18, weight='bold', pad=20)
    ax1.legend(loc='upper right', fontsize=13, framealpha=0.95)
    ax1.grid(True, alpha=0.3, linestyle='--', linewidth=0.5)
    
    plt.tight_layout()
    
    image_file_1 = os.path.join(output_folder, 'optimization_path.png')
    plt.savefig(image_file_1, dpi=300, bbox_inches='tight')
    plt.close(fig1)
    
    fig2 = plt.figure(figsize=(14, 10))
    ax2 = fig2.add_subplot(1, 1, 1)
    
    ax2.semilogy(stats['best_values'], 'b-', linewidth=3, label='Best Function Value')
    ax2.set_xlabel('Iteration', fontsize=16, weight='bold')
    ax2.set_ylabel('Best Function Value (log scale)', fontsize=16, weight='bold')
    ax2.set_title(f'{algorithm_name} - Convergence History', fontsize=18, weight='bold', pad=20)
    ax2.grid(True, alpha=0.4, linestyle='--', linewidth=1)
    ax2.axhline(y=1e-10, color='r', linestyle='--', linewidth=2.5, label='Target: 1e-10')
    ax2.legend(fontsize=14, framealpha=0.95)
    
    convergence_text = f"CONVERGENCE INFO\n"
    convergence_text += f"{'='*30}\n"
    convergence_text += f"Initial Value: {stats['best_values'][0]:.2e}\n"
    convergence_text += f"Final Value: {stats['best_values'][-1]:.2e}\n"
    convergence_text += f"Improvement: {stats['best_values'][0] - stats['best_values'][-1]:.2e}\n"
    convergence_text += f"Total Iterations: {len(stats['best_values'])}"
    
    ax2.text(0.02, 0.98, convergence_text, transform=ax2.transAxes, fontsize=11,
            verticalalignment='top', bbox=dict(boxstyle='round', facecolor='lightblue', alpha=0.85),
            family='monospace')
    
    plt.tight_layout()
    
    image_file_2 = os.path.join(output_folder, 'convergence_history.png')
    plt.savefig(image_file_2, dpi=300, bbox_inches='tight')
    plt.close(fig2)

def generate_report(results_custom, results_scipy, dimensions, use_scipy, output_folder, algorithm_name):
    report_lines = []
    report_lines.append("="*80)
    report_lines.append(f"{algorithm_name.upper()} OPTIMIZATION REPORT - RASTRIGIN FUNCTION")
    report_lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    report_lines.append("="*80)
    report_lines.append("")
    
    for dim in dimensions:
        report_lines.append(f"\n{'='*80}")
        report_lines.append(f"DIMENSION: {dim}D")
        report_lines.append(f"{'='*80}")
        
        result = results_custom[dim]
        x0 = result['x0']
        x_best = result['x_best']
        f_best = result['f_best']
        stats = result['stats']
        
        report_lines.append(f"\nRandom Initial Point: {x0}")
        report_lines.append(f"Initial Function Value: {rastrigin(x0):.6f}")
        
        report_lines.append(f"\n{'-'*80}")
        report_lines.append(f"{algorithm_name.upper()} (CUSTOM IMPLEMENTATION)")
        report_lines.append(f"{'-'*80}")
        
        report_lines.append(f"Found Minimum: {x_best}")
        report_lines.append(f"Function Value: {f_best:.10f}")
        report_lines.append(f"Distance from True Minimum: {np.linalg.norm(x_best):.10f}")
        report_lines.append(f"Number of Function Evaluations: {stats['nfev']}")
        report_lines.append(f"Path Length: {stats['path_length']} points")
        report_lines.append(f"Accepted Moves: {stats['accepted_count']}")
        report_lines.append(f"Rejected Moves: {stats['rejected_count']}")
        report_lines.append(f"Acceptance Rate: {stats['acceptance_rate']:.4f}")
        
        if use_scipy and dim in results_scipy:
            result_scipy = results_scipy[dim]
            report_lines.append(f"\n{'-'*80}")
            report_lines.append("SCIPY LIBRARY (COMPARISON)")
            report_lines.append(f"{'-'*80}")
            
            report_lines.append(f"Found Minimum: {result_scipy['x_best']}")
            report_lines.append(f"Function Value: {result_scipy['f_best']:.10f}")
            report_lines.append(f"Distance from True Minimum: {np.linalg.norm(result_scipy['x_best']):.10f}")
            report_lines.append(f"Number of Function Evaluations: {result_scipy['nfev']}")
            
            report_lines.append(f"\n{'-'*80}")
            report_lines.append("COMPARISON")
            report_lines.append(f"{'-'*80}")
            report_lines.append(f"Function Value Difference: {abs(f_best - result_scipy['f_best']):.10e}")
            report_lines.append(f"Solution Distance: {np.linalg.norm(x_best - result_scipy['x_best']):.10e}")
    
    report_lines.append(f"\n\n{'='*80}")
    report_lines.append("SUMMARY TABLE")
    report_lines.append(f"{'='*80}")
    if use_scipy:
        report_lines.append(f"{'Dim':<6} {'Custom f(x)':<18} {'SciPy f(x)':<18} {'Custom nfev':<14} {'SciPy nfev':<14}")
    else:
        report_lines.append(f"{'Dim':<6} {'Custom f(x)':<18} {'Custom nfev':<14} {'Accept Rate':<14}")
    report_lines.append(f"{'-'*80}")
    for dim in dimensions:
        if use_scipy and dim in results_scipy:
            report_lines.append(f"{dim:<6} {results_custom[dim]['f_best']:<18.10f} {results_scipy[dim]['f_best']:<18.10f} "
                  f"{results_custom[dim]['stats']['nfev']:<14} {results_scipy[dim]['nfev']:<14}")
        else:
            report_lines.append(f"{dim:<6} {results_custom[dim]['f_best']:<18.10f} "
                  f"{results_custom[dim]['stats']['nfev']:<14} {results_custom[dim]['stats']['acceptance_rate']:<14.4f}")
    
    report_text = "\n".join(report_lines)
    
    report_file = os.path.join(output_folder, 'optimization_report.txt')
    with open(report_file, 'w') as f:
        f.write(report_text)
    
    return report_text