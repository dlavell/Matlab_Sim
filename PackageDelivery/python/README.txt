README for the Python Module

**NOTE: python.m calls on the relative location of python_library. The
library is also neccessary for the python code to run.

===== FILES/FOLDERS ===== 
* python_library/
This folder contains the python executable and all associated libraries to
run the raster image reading functions. 

* get_elevation.m
This is the MATLAB wrapper function to call the python library on the
python script.

* python.m
This is a MATLAB script to run python code by calling the python library.
There is a dependency on python_library's code and relative location in 
order to run this script.

* python_get_elevation.py
This is the actual python code that does all of the raster image reading.
