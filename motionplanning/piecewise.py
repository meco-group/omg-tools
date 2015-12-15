from numpy.polynomial import Polynomial


class Piecewise(object):
    """
    General implementation of piecewise functions
    """
    def __init__(self, knots, functions):
        self.knots = knots
        self.functions = functions

    def __str__(self):
        return "Piecewise function with intervals %s and functions %s" % (
            self.knots, self.functions)

        __unicode__ = __str__

    def __call__(self, x):
        """
        Evaluate the piecewise function at x. Assumes the functions are callable
        """
        return [f(x[i]) for i, f in enumerate(map(self._which_function, x))]

    def domain(self):
        """
        Returns the domain on which the function is defined
        """
        return (min(self.knots), max(self.knots))

    def _which_function(self, x):
        """
        Returns the function to evaluate self at x.
        Returns 0 when outside the domain
        TODO: do not loop over all intervals
        """
        for i, (a, b) in enumerate(zip(self.knots[:-1], self.knots[1:])):
            if a <= x < b:
                return self.functions[i]
        return 0

    def _combine_knots(self, other):
        """
        Returns the union of the intervals of self and other
        """
        knots = list(set(self.knots).union(set(other.knots)))
        knots.sort()
        return knots

    def __add__(self, other):
        if not isinstance(other, Piecewise):
            return Piecewise(self.knots,
                             [other + f for f in self.functions])
        knots = self._combine_knots(other)
        functions = []
        for (a, b) in zip(knots[:-1], knots[1:]):
            x = 0.5 * (a + b)
            functions.extend([self._which_function(x) +
                              other._which_function(x)])
        return Piecewise(knots, functions)

    def __mul__(self, other):
        # Scalar multiplication
        if not isinstance(other, Piecewise):
            return Piecewise(self.knots,
                             [other * f for f in self.functions])
        # Piecewise multiplication
        knots = self._combine_knots(other)
        functions = []
        for (a, b) in zip(knots[:-1], knots[1:]):
            x = 0.5 * (a + b)
            functions.extend([self._which_function(x) *
                              other._which_function(x)])
        return Piecewise(knots, functions)

    __rmul__ = __mul__


class PiecewisePolynomial(Piecewise):
    """
    Subclass of Piecewise. Functions are limited to Polynomials
    """
    def __init__(self, knots, functions):
        self.knots = knots
        self.functions = map(Polynomial, functions)
