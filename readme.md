<img width=850 src="https://cdn.rawgit.com/meco-group/omg-tools/master/doc/banner.svg" alt="oh my god!"/>

[![Build Status](https://travis-ci.org/meco-group/omg-tools.svg?branch=master)](https://travis-ci.org/meco-group/omg-tools) [![Coverage Status](https://coveralls.io/repos/github/meco-group/omg-tools/badge.svg?branch=master)](https://coveralls.io/github/meco-group/omg-tools?branch=master)
[![Codacy Badge](https://api.codacy.com/project/badge/grade/a44dd52c358e4cd09b12585915738627)](https://www.codacy.com/app/joris-gillis42/omg-tools)
[![PyPI version](https://badge.fury.io/py/omg-tools.svg)](https://badge.fury.io/py/omg-tools)

Optimal Motion Generation-tools is a Python software toolbox facilitating the modeling, simulation and embedding of motion planning problems. Its main goal is to collect research topics concerning (spline-based) motion planning into a user-friendly package in order to enlarge its visibility towards the scientific and industrial world.

This toolbox focuses on receding horizon control for single-agent systems as well as on distributed control for multi-agent systems. The approaches implemented in OMG-tools are described in the following publications:
* Mercy T., Van Loock W., Pipeleers G. (2016). Real-time motion planning in the presence of moving obstacles. Proceedings of the 2016 European Control Conference. European Control Conference. Aalborg, 29 June - 1 July 2016 (pp. 1586-1591). ([pdf](https://lirias.kuleuven.be/bitstream/123456789/538718/1/TimMercy_2016_ECC.pdf))
* Van Parys R., Pipeleers G. (2016). Online distributed motion planning for multi-vehicle systems. Proceedings of the 2016 European Control Conference. European Control Conference. Aalborg, 29 June - 1 July 2016 (pp. 1580-1585). ([pdf](https://lirias.kuleuven.be/bitstream/123456789/526758/3/RubenVanParys_2016_ECC.pdf))
* Van Parys R., Pipeleers G. (2017). Spline-Based Motion Planning in an Obstructed 3D environment. Proceedings of the 20th IFAC World Congress. IFAC World Congress. Toulouse, France, 9-14 July 2017. ([pdf](https://lirias.kuleuven.be/bitstream/123456789/575111/3/RubenVanParys_2017_IFAC_lirias.pdf))

If these methods help you with your research, please cite us!

## Examples
### Overview
The animations below give an overview of typical problems that OMG-tools can handle.

<table style="border: none; border-collapse: collapse;" border="0" cellspacing="0" cellpadding="0" width="100%" align="center">
<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<img width=100% src="./doc/gifs/bicycle.gif" alt="Point-to-point motion of bicycle model"/>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/maze.gif" alt="Holonomic vehicle moving through a maze"/>
</td>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<img width=100% src="./doc/gifs/revolving_door.gif" alt="Holonomic vehicle passing through a revolving door"/>
</td>
<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<img width=100% src="./doc/gifs/formation_quad_rotatingwall.gif" alt="Formation of quadrotor with rotating wall"/>
</td>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<img width=100% src="./doc/gifs/formation_dubins.gif" alt="Dubins vehicles moving in relative formation"/>
</td>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<img width=100% src="./doc/gifs/trailer.gif" alt="Dubins vehicle with trailer"/>
</td>
</tr>
<tr>
<td aliggn="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/multiproblem_rect.gif" alt="Big warehouse with rectangular bouncing obstacles"/>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/formation_quad_ufo.gif" alt="Quadrotors avoiding a UFO"/>
</td>
<td aliggn="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/3dquadrotor.gif" alt="3D quadrotor in obstructed environment"/>
</td>
</tr>
<tr>
<td align="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/agv.gif" alt="Rear-wheel steered AGV parking"/>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/p2p_holonomic_blocking.gif" alt="Holonomic vehicle moving while suddenly blocked"/>
</td>
<td aliggn="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/revolving_door_diffdrive.gif" alt="Differential drive moving through a revolving door"/>
</td>
</tr>
<tr>
<td align="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/multiproblem_circ.gif" alt="Big warehouse with circular bouncing obstacles"/>
</td>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<img width=100% src="./doc/gifs/warehouse.gif" alt="Holonomic vehicle finding its way in a warehouse"/>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<img width=100% src="./doc/gifs/platform_landing.gif" alt="Quadrotors landing on platform"/>
</td>
</tr>
</table>

### Experimental validation
OMG-tools implemented on real-life motion systems. Click on a picture to watch the Youtube video.

<table style="border: none; border-collapse: collapse;" border="0" cellspacing="0" cellpadding="0" width="100%" align="center">
<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<a href="https://www.youtube.com/watch?v=w7tfz2djHqU">
<img src="https://img.youtube.com/vi/w7tfz2djHqU/0.jpg" alt="Online motion planning"/>
</a>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<a href="https://www.youtube.com/watch?v=J_ShOP_VWTg">
<img src="https://img.youtube.com/vi/J_ShOP_VWTg/0.jpg" alt="Online distributed motion planning"/>
</a>
</td>
</tr>

<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<a href="https://www.youtube.com/watch?v=ozV3aJvEAyY">
<img src="https://img.youtube.com/vi/ozV3aJvEAyY/0.jpg" alt="Spline-based motion planning"/>
</a>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<a href="https://www.youtube.com/watch?v=yGGZv57eiB4">
<img src="https://img.youtube.com/vi/yGGZv57eiB4/0.jpg" alt="Spline-based motion planning"/>
</a>
</td>
</tr>

<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<a href="https://www.youtube.com/watch?v=4JaY0_k24WM">
<img src="https://img.youtube.com/vi/4JaY0_k24WM/0.jpg" alt="Spline-based motion planning"/>
</a>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<a href="https://www.youtube.com/watch?v=0ftMz0JTojE">
<img src="https://img.youtube.com/vi/0ftMz0JTojE/0.jpg" alt="Plate transportation"/>
</a>
</td>
</tr>

<tr>
<td align="center" valign="center" style="background-color:rgba(0, 0, 0, 0);">
<a href="https://www.youtube.com/watch?v=WIt9lRXMW-4">
<img src="https://img.youtube.com/vi/WIt9lRXMW-4/0.jpg" alt="Spline-based motion planning in a vast environment"/>
</a>
</td>
<td align="center" valign="center" bgcolor="#FFFFFF">
<a href="https://www.youtube.com/watch?v=3nlfVI4L3p8">
<img src="https://img.youtube.com/vi/3nlfVI4L3p8/0.jpg" alt="Spline-based motion planning in a vast environment"/>
</a>
</td>
</tr>

</table>


### Code example
This elementary code example illustrates the basic functionality of the toolbox for steering a holonomic vehicle from an initial to terminal pose in a dynamic environment.

```python
from omgtools import *

# make and set-up vehicle
vehicle = Holonomic()
vehicle.set_initial_conditions([-1.5, -1.5])
vehicle.set_terminal_conditions([2., 2.])
vehicle.set_options({'safety_distance': 0.1})

# make and set-up environment
environment = Environment(room={'shape': Square(5.)})

# add stationary obstacles to environment
rectangle = Rectangle(width=3., height=0.2)
environment.add_obstacle(Obstacle({'position': [-2.1, -0.5]}, shape=rectangle))
environment.add_obstacle(Obstacle({'position': [ 1.7, -0.5]}, shape=rectangle))

# generate trajectory for moving obstacle
traj = {'velocity': {'time': [3., 4.],
                     'values': [[-0.15, 0.0], [0., 0.15]]}}
# add moving obstacle to environment
environment.add_obstacle(Obstacle({'position': [1.5, 0.5]}, shape=Circle(0.4),
    simulation={'trajectories': traj}))

# give problem settings and create problem
problem = Point2point(vehicle, environment)
problem.init()

# simulate, plot some signals and save a movie
simulator = Simulator(problem)
vehicle.plot('input', labels=['v_x (m/s)', 'v_y (m/s)'])
problem.plot('scene')
simulator.run()
problem.save_movie('scene')
```
<p align="center">
<img width=400 src="./doc/gifs/p2p_holonomic.gif" alt="Point-to-point motion of holonomic vehicle"/>
</p>

### More examples
Check out the examples directory for more code examples. There you can find a simple tutorial example which provides a documented overview of the basic functionality of the toolbox.

## Installation
### Basic installation
OMG-tools is written in Python 2.7 and requires the installation of the following packages:

`sudo apt-get install python-pip python-numpy python-scipy python-matplotlib`

OMG-tools itself is downloaded from the PyPI repository and installed using `pip`:

`sudo pip install omg-tools`

### Advanced installation
If you want to save simulation results in gif-format, you need [imagemagick](www.imagemagick.org):

`sudo add-apt-repository main && apt-get update && install imagemagick `

For faster solving of the motion problems, we recommend to install the [HSL linear solvers](https://github.com/casadi/casadi/wiki/Obtaining-HSL).

If you want to install OMG-tools for development purposes, you can link your installed files to the cloned repository, by invoking the following command in the repository's root folder:

`sudo pip install -e .`


## Authors
OMG-tools is developed by Ruben Van Parys and Tim Mercy as part of their research in spline-based motion planning, under supervision of Goele Pipeleers within the [MECO research team](https://www.mech.kuleuven.be/en/pma/research/meco). Any questions, comments or propositions of collaboration can be addressed to ruben[dot]vanparys[at]kuleuven[dot]be and tim[dot]mercy[at]kuleuven[dot]be.
