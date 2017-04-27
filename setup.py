from setuptools import setup, find_packages

setup(
    name='omg-tools',
    version='0.1.1',
    author='Ruben Van Parys',
    author_email='ruben.vanparys@kuleuven.be',
    description=('Optimal Motion Generation tools: a user-friendly tool for ' +
                 'modeling, simulating and embedding of (spline-based) motion ' +
                 'planning problems'),
    long_description=('Optimal Motion Generation-tools is a Python software ' +
                      'toolbox facilitating the modeling, simulation and ' +
                      'embedding of motion planning problems. ' +
                      'Its main goal is to collect research topics ' +
                      'concerning (spline-based) motion planning into a ' +
                      'user-friendly package in order to enlarge its ' +
                      'visibility towards the scientific and industrial world.'),
    license='LGPLv3',
    keywords='optimization motion planning splines distributed multi-agent mpc',
    url='https://github.com/meco-group/omg-tools',
    packages=find_packages(),
    package_data={'omgtools.export': ['*/Makefile', '*/instructions.txt',
                                      '*.cpp', '*.hpp',
                                      '*/*.cpp', '*/*.hpp',
                                      '*/*/*.cpp', '*/*/*.hpp',
                                      '*/*/*/*.cpp', '*/*/*/*.hpp']},
    install_requires=[
        'numpy',
        'scipy',
        'matplotlib',
        'matplotlib2tikz',
        'casadi >= 3.1.0'
        ],
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Science/Research',
        'Topic :: Scientific/Engineering',
        'License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)',
        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
    ],
)
