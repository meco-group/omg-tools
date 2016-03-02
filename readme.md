# Spline Based Motion Planning Toolbox
We should think about a more attractive name...
Something with move? plan? 
Should spline be in there?

## Its purpose?
This software package provides tools that make it easy to model, simulate and embed motion planning problems. The main goal is to bring several research topics and several techniques concerning (spline-based) motion planning together into a user-friendly package.

## How to start?
The software is developed in Python and uses [CasADi](https://github.com/casadi/casadi/wiki) as a framework for symbolic computations. Furthermore CasADi provides an interface to IPOPT, a software package for large-scale nonlinear optimization. For installation instructions regarding these software packages, the user is referred to the [CasADi homepage](https://github.com/casadi/casadi/wiki), and more in particular the installation instructions page.
With the toolbox it is possible to save your simulations in Tikz format. For this functionality uses the following script: [matplotlib2tikz](https://github.com/nschloe/matplotlib2tikz).
To install the toolbox itself, run:
    `>> sudo python setup.py install`

## How does it work?
The best way to get started with the toolbox is by studying the examples map, and more in particular the file `tutorial_example.py`. This file provides a well-documented overview of the basic functionality of the toolbox.
