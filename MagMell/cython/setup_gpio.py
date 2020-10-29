from setuptools import setup
from Cython.Build import cythonize

setup(
    name='Gpio',
    ext_modules=cythonize(['DRV_GPIO.pyx'])
)
