# Spline Based Motion Planning Toolbox
We should think about a more attractive name...

## Its purpose?
This software package provides tools that facilitate the modeling, simulation and embedding of motion planning problems. The main goal is bringing related research concerning (spline-based) motion planning together into a user-friendly package.

## How to start?
The software is developed in Python and uses [CasADi](https://github.com/casadi/casadi/wiki) as a framework for symbolic computations. Furthermore CasADi provides an interface to IPOPT, a software package for large-scale nonlinear optimization. For installation instructions regarding these software packages, the user is referred to the [CasADi homepage](https://github.com/casadi/casadi/wiki).
The toolbox provides functionality to save simulation results in Tikz format. This functionality depends on [matplotlib2tikz](https://github.com/nschloe/matplotlib2tikz).
The toolbox itself can be installed by running
    `>> sudo python setup.py install`

## How does it work?
Before writing down a fancy tutorial here, we refer the user to the examples directory. `tutorial_example.py` provides a well-documented overview of the basic functionality of the toolbox.
