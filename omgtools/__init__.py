from .vehicles import *
from .environment import *
from .basics import *
from .problems import *
from .execution import *
from .export import *
# from .gui import *


def assert_ma57():
  import casadi
  import sys

  x = casadi.MX.sym("x")
  nlp = casadi.nlpsol('nlp','ipopt',{"x":x,"f":x**2},{"ipopt": {"linear_solver": "ma57", "print_level":0}, "print_time": False})

  nlp()

  if nlp.stats()["return_status"]=="Invalid_Option":
    print("Could not find ma57 (from hsl library), which is needed for the code to run.")
    sys.exit(0)
