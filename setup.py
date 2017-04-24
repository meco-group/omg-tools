import os
from setuptools import setup


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name='omg-tools',
    version='0.1',
    author='Ruben Van Parys',
    author_email='ruben.vanparys@kuleuven.be',
    description=('Optimal Motion Generation tools: a user-friendly tool for ' +
                 'modeling, simulating and embedding of (spline-based) motion ' +
                 'planning problems'),
    license="LGPLv3",
    keywords='optimization motion planning splines distributed multi-agent',
    url='https://github.com/meco-group/omg-tools',
    packages=['omgtools', 'omgtools/basics', 'omgtools/environment',
              'omgtools/problems', 'omgtools/execution', 'omgtools/vehicles',
              'omgtools/export'],
    package_data={'': ['export/point2point/Makefile',
                       'export/point2point/instructions.txt',
                       'export/*.cpp', 'export/*.hpp',
                       'export/*/*.cpp', 'export/*/*.hpp',
                       'export/*/*/*.cpp', 'export/*/*/*.hpp',
                       'export/*/*/*/*.cpp', 'export/*/*/*/*.hpp']},
    long_description=read('readme.md'),
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
    ],
)
