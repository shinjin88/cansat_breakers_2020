from setuptools import setup
from Cython.Build import cythonize

setup(
    name='GPS_Serial',
    ext_modules=cythonize(['DRV_GPS_Serial.pyx'])
)
