from point2pointproblem import FreeTPoint2point, FixedTPoint2point


class Point2point(object):

    # This class looks at the problem type of the created problem and
    # creates the corresponding instance (freeTProblem or fixedTProblem)
    def __new__(cls, fleet, environment, options={}):
        if (options['freeTProblem'] is True):
            # construct FreeTPoint2point instance
            return FreeTPoint2point(fleet, environment, options)
        elif (options['freeTProblem'] is False):
            # construct FixedTPoint2point instance
            return FixedTPoint2point(fleet, environment, options)
        else:
            raise ValueError('Problem type not specified! Pass the option '
                             'freeTProblem (boolean) to Point2Point to '
                             'select the freeT or fixedT problem type.')

    def __init__(self, fleet, environment, options={}):
        self.options = options
