## test examples
* p2p_holonomic.py
* revolving_door.py
* p2p_dubins.py
* formation_holonomic.py
* formation_dubins.py
* p2p_3dquadrotor.py
* platform_landing.py
* p2p_bicycle.py

## user stories: desired functionality from omg-tools side

1) general spline construction: from user-defined
    * coefficients: numeric/symbolic
    * knot sequence: numeric/symbolic
    * degree: numeric
  separation between basis and function construction is desired.
  motivation for symbolics:
    * coefficients: used as variables
    * knot sequence: 1st (degree+1) knots used as parameter (current time), that changes over MPC iterates.
  all further defined manipulations are desirable to work for numeric/symbolic coefficients and knots

2) on splines one can execute basic arithmitic operation:
    * add, sub, neg, mul, pow
    * both between 2 splines and spline/scalar
    * desired as operation overloading for increasing code-readability

3) on spline one can execute basic analytic operations:
    * derivative(o=1)
    * antiderivative(o=1)
    * integral(lb, ub): integral over part of domain ([lb,ub])

4) functionality to insert (multiple) knots:
    * spline.insert_knots(knots)
    * inserted knots can be symbolic
    * transformation matrix accessible from BSplineBasis:
      - basis.get_insert_knots_tf(knots)
    * required by crop functionality

5) one can crop a spline
    * spline.crop(lb, ub)
    * returns a spline defined over cropped domain
      - keeping the knot sequence within the cropped domain
      - using (degree+1) knots at begin and end of cropped domain
    * lb and ub can be symbolic
    * based on knot insertion at bounds of domain
    * transformation matrix accessible from BSplineBasis:
      - basis.get_crop_tf(knots)
    * required by omg-tools functionality for transforming splines between MPC horizons (using fixedT)

6) one can extrapolate a spline
    * spline.extrapolate(t_extra, m=None)
    * extrapolates spline by adding (degree+1) knots t_extra further than given last knot, leaves m internal knots at this knot and imposes continuity at this knot up to the degreeth derivative
    * transformation matrix accessible from BSplineBasis:
      - basis.get_extrapolate_tf(t_extra, m=None)
    * required by omg-tools functionality for transforming splines between MPC horizons (using fixedT)

7) one can transform between bases (exact and inexact)
    * basis2.transform(basis1)
    * return transformation matrix for transforming coeffs between 2 bases
    * this is least-squares bases (see implementation of splines-py)
    * required by omg-tools functionality for transforming splines between MPC horizons (using freeT)

## priorities

* Highest: arithmitic operations + analytic operations
* highest: insert_knot, crop, extrapolate, transform
* lowest: symbolic knots (omg-tools is not prepared for this)
