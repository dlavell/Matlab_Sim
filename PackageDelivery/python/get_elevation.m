function [elevation] = get_elevation(lat, lng)
%GET_ELEVATION This is a wrapper function for the python_get_elevation
%function.
%Input: latitude and longitude of desired elevation
%Output: returns the elevation from the read raster image

[result,] = python('python_get_elevation.py', num2str(lat), num2str(lng));
elevation = str2double(result);
end

