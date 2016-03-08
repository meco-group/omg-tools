import os
from setuptools import setup


def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()


setup(
    name="omg-tools",
    version="0.1",
    author="Ruben Van Parys & Tim Mercy",
    author_email="ruben.vanparys@kuleuven.be",
    description=('optimal motion generation tools: a user-friendly tool for ' +
                 'modeling, simulating and embedding of (spline-based) motion ' +
                 'planning problems'),
    license="LGPLv3",
    keywords="optimization motion planning splines distributed multi-agent",
    # url="",
    packages=['omgtools'],
    package_data={'':['export/point2point/*']},
    long_description=read('readme.md'),
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
    ],
)
