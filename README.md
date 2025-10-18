# Global Optimization Algorithms for Rastrigin Function

Complete implementation and comparison of Dual Annealing and Basin Hopping algorithms for solving highly multimodal optimization problems.

---

## Table of Contents

1. [Overview](#overview)
2. [File Structure](#file-structure)
3. [Prerequisites](#prerequisites)
4. [Quick Start](#quick-start)
5. [Parameter Configuration](#parameter-configuration)
6. [Output Files](#output-files)

---

## Overview

This project implements two state-of-the-art global optimization algorithms:

- **Dual Annealing**: Combines Generalized Simulated Annealing (GSA) with local search, using Tsallis statistics and fat-tailed Cauchy-Lorentz distribution for efficient exploration.
- **Basin Hopping**: Transforms the energy landscape by performing local minimization after random perturbations, using Metropolis criterion for acceptance.

Both algorithms are tested on the **Rastrigin function**, a highly multimodal benchmark function with numerous local minima.

**Key Features:**
- Custom Python implementations from scratch
- SciPy library examples for comparison
- Automatic visualization and report generation
- Multi-dimensional testing (2D-6D)
- Reproducible results with fixed initial points

---

## File Structure
```
├── dual_annealing.py           # SciPy Dual Annealing example
├── basin_hopping.py            # SciPy Basin Hopping example
├── dual_annealing_custom.py    # Custom Dual Annealing implementation
├── basin_hopping_custom.py     # Custom Basin Hopping implementation
├── visualize.py                # Visualization and report generation utilities
└── README.md                   # This file
```

---

## Prerequisites

### Install Dependencies
```bash
pip install numpy scipy matplotlib
```
---

## Quick Start

### Step 1: Simple Testing with SciPy

Test with a single dimension using library implementations:

**Dual Annealing:**
```bash
python dual_annealing.py
```

**Basin Hopping:**
```bash
python basin_hopping.py
```

### Step 2: Full Experiments with Custom Implementation

Run complete experiments across multiple dimensions:

**Dual Annealing:**
```bash
python dual_annealing_custom.py
```

**Output:** Creates `dual_annealing/` folder with:
- `optimization_path.png` - 2D visualization of search path
- `convergence_history.png` - Convergence over iterations
- `optimization_report.txt` - Detailed results for all dimensions

**Basin Hopping:**
```bash
python basin_hopping_custom.py
```

**Output:** Creates `basin_hopping/` folder with similar files.

---

## Parameter Configuration

### Simple Examples (`dual_annealing.py` & `basin_hopping.py`)

#### Change Problem Dimension
```python
dimension = 2  # Change to 3, 4, 5, 10, etc.
```

#### Set Initial Point
```python
x0 = np.random.uniform(-5.12, 5.12, dimension)

# OR - Fixed start
x0 = np.ones(dimension) * 2.5
```

#### Dual Annealing Parameters
```python
result = dual_annealing(
    rastrigin,
    bounds,
    x0=x0,
    maxiter=1000,           # Number of global iterations
    initial_temp=5230,      # Initial temperature (default: 5230)
    visit=2.62,             # Visiting parameter qv ∈ (1, 3]
    accept=-5.0             # Acceptance parameter qa ∈ (-∞, -5]
)
```

**Parameter Guide:**
- `maxiter`: More iterations → better convergence, longer runtime
- `initial_temp`: Higher → more exploration, slower convergence
- `visit`: Higher (→3) → longer jumps, more exploration
- `accept`: More negative → more selective acceptance

#### Basin Hopping Parameters
```python
# Lines 20-24
result = basinhopping(
    rastrigin,
    x0,
    niter=500,              # Number of basin hopping iterations
    T=2.0,                  # Temperature for acceptance
    stepsize=1.5,           # Maximum perturbation step size
    minimizer_kwargs=minimizer_kwargs
)
```

**Parameter Guide:**
- `niter`: More iterations → better exploration
- `T`: Higher → accept worse solutions more easily
- `stepsize`: Larger → bigger jumps between basins

---

### Custom Implementations (`*_custom.py`)

#### Change Test Dimensions
```python
dimensions = [2, 3, 4, 5, 6]  # Modify this list
```

#### Change Initial Point Strategy
```python
# Option 1: Fixed start
x0 = np.ones(dim) * 2.5

# Option 2: Random start
x0 = np.random.uniform(-5.12, 5.12, dim)
```

#### Dual Annealing Parameters
```python
optimizer = DualAnnealing(
    rastrigin, 
    bounds, 
    initial_temp=5230.0,    # Initial temperature
    visit=2.62,             # Visiting parameter qv
    accept=-5.0,            # Acceptance parameter qa
    maxiter=1000            # Maximum iterations
)
```

#### Basin Hopping Parameters
```python
optimizer = BasinHopping(
    rastrigin, 
    bounds, 
    niter=500,              # Number of iterations
    T=2.0,                  # Temperature parameter
    stepsize=1.5            # Step size for perturbation
)
```

---

## Output Files

### Generated Files (Custom Implementations)

#### 1. `optimization_path.png`
- 2D contour plot of Rastrigin function
- Optimization path with numbered steps
- Starting point (red star) and final minimum (green triangle)
- Statistics box with performance metrics

#### 2. `convergence_history.png`
- Log-scale plot of best function value over iterations
- Shows convergence speed
- Target line at 1e-10

#### 3. `optimization_report.txt`
Complete text report including:
- Initial and final function values
- Distance from true global minimum
- Number of function evaluations
- Path statistics (accepted/rejected moves, acceptance rate)
- Summary table for all dimensions
- Comparison with SciPy (if enabled)

---

### Performance Notes

- Results may vary slightly due to floating-point precision
- Higher dimensions require more function evaluations
- Dual Annealing uses adaptive temperature scheduling
- Basin Hopping uses adaptive step size adjustment

---

## Academic Use

These implementations were developed for:

**Course:** FRA502 - Advanced Robotic Control and Simulation with ROS2 (ARCS2)  
**Institution:** King Mongkut's University of Technology Thonburi (KMUTT)  
**Program:** Field Robotics and Automation Engineering

---

**Last Updated:** October 2025  
**Version:** 1.0  
**Tested On:** Python 3.13, Windows 11
