from setuptools import setup
from Cython.Build import cythonize

setup(
    name='LibIMU',
    ext_modules=cythonize(['LibIMU.pyx'])
)
