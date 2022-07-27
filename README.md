# AutoGenU for Jupyter

[![Build Status](https://travis-ci.org/mayataka/autogenu-jupyter.svg?branch=master)](https://travis-ci.org/mayataka/autogenu-jupyter) 
[![Build status](https://ci.appveyor.com/api/projects/status/s12498n4fd847sqf?svg=true)](https://ci.appveyor.com/project/mayataka/autogenu-jupyter)

## Introduction
This project provides the continuation/GMRES method (C/GMRES method) based solvers for nonlinear model predictive control (NMPC) and an automatic code generator for NMPC, called AutoGenU.

The following C/GMRES based solvers are provided: 
- `SingleShootingCGMRESSolver` : The original C/GMRES method (single shooting).
- `MultipleShootingCGMRESSolver` : The multiple shooting based C/GMRES method with condensing of the state and the Lagragne multipliers with respect to the state equation.

## Requirement
- C++17 (MinGW or MSYS and PATH to either are required for Windows users)
- CMake
- Python 3.8 or later, Jupyter Lab or Jupyter Notebook, SymPy (to generate `ocp.hpp`, `main.cpp`, and `CMakeLists.txt` by `AutoGenU.ipynb`)
- Python 3.8 or later, NumPy, Matplotlib, seaborn (to plot simulation data on `AutoGenU.ipynb`)
- ffmpeg (to generate animations in `pendubot.ipynb`, `cartpole.ipynb`, `hexacopter.ipynb`, and `mobilerobot.ipynb`)

## Usage
### Submodules 
Please confirm that you clone this repository as 
```
git clone https://github.com/mayataka/autogenu-jupyter --recursive
```
Otherwise, please do the following command:
```
git submodule update --init --recursive
```

### AutoGenU
`AutoGenU.ipynb` generates following source files under your setting state equation, constraints, cost function, and parameters: 
- `ocp.hpp`  
- `main.cpp`  
- `CMakeLists.txt`

You can also build source files for numerical simulation, execute numerical simulation, and plot or save simulation result on `AutoGenU.ipynb`.


### C/GMRES based solvers of NMPC
The C/GMRES based solvers in `src/solver` directory can be used independently of `AutoGenU.ipynb`. You are then required the following files:
- `ocp.hpp`: write the optimal control problem (OCP) in your model  

In addition to these files, you have to write `CMakeLists.txt` to build source files.


## Demos
Demos are presented in `pendubot.ipynb`, `cartpole.ipynb`, `hexacopter.ipynb`, and `mobilerobot.ipynb`. You can obtain the following simulation results jusy by runnig these `.ipynb` files. The details of the each OCP formulations are described in each `.ipynb` files.

### Pendubot  
Inverting a pendubot using `MultipleShootingCGMRESSolver` solver.
![pendubot_gif](https://raw.githubusercontent.com/wiki/mayataka/CGMRES/images/pendubot.gif)
![pendubot_png](https://raw.github.com/wiki/mayataka/CGMRES/images/pendubot.png)

### Cartpole
Inverting a cartpole using `SingleShootingCGMRESSolver` solver.
![cartpole_gif](https://raw.githubusercontent.com/wiki/mayataka/CGMRES/images/cartpole.gif)
![cartpole_png](https://raw.github.com/wiki/mayataka/CGMRES/images/cartpole.png)

### Hexacopter 
Trajectory tracking of a hexacopter using `MultipleShootingCGMRESSolver` .
![hexacopter_gif](https://raw.githubusercontent.com/wiki/mayataka/CGMRES/images/hexacopter.gif)
![hexacopter_png](https://raw.github.com/wiki/mayataka/CGMRES/images/hexacopter.png)

### Mobile robot
Obstacle avoidance of a mobile robot using `MultipleShootingCGMRESSolver` solver with the semi-smooth Fischer-Burmeister method for inequality constraints.
![mobilerobot_gif](https://raw.githubusercontent.com/wiki/mayataka/CGMRES/images/mobilerobot.gif)
![mobilerobot_png](https://raw.github.com/wiki/mayataka/CGMRES/images/mobilerobot.png)


## License
MIT

## References
1. [T. Ohtsuka A continuation/GMRES method for fast computation of nonlinear receding horizon control, Automatica, Vol. 40, No. 4, pp. 563-574 (2004)](https://doi.org/10.1016/j.automatica.2003.11.005)
2. [C. T. Kelly, Iterative methods for linear and nonlinear equations, Frontiers in Apllied Mathematics, SIAM (1995)](https://doi.org/10.1137/1.9781611970944)
3. [Y. Shimizu, T. Ohtsuka, M. Diehl, A real‐time algorithm for nonlinear receding horizon control using multiple shooting and continuation/Krylov method, International Journal of Robust and Nonlinear Control, Vol. 19, No. 8, pp. 919-936 (2008)](https://doi.org/10.1002/rnc.1363)
