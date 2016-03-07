"""
WorldEngine is used as the data layer for the threaded server. Retrieves and processes data stored in raster images.
Manages
"""
from __future__ import absolute_import, division, print_function, unicode_literals
from builtins import object
from builtins import str

__metaclass__ = type
import numpy as np
import rasterio
import sys
from collections import namedtuple
from numpy import NaN, math

try:
    from osgeo import gdal
except ImportError:
    import osgeo.gdal as gdal

try:
    from osgeo import osr
except ImportError:
    import osgeo.gdal as osr

Coordinate = namedtuple("Coordinate", ['lat', 'lon'], verbose=False)
PixelPair = namedtuple("PixelPair", ['x', 'y'], verbose=False)


class ReadException(Exception):
    """

    ReadExceptions occur in response to invalid rasters or file paths

    """

    def __init__(self, strn):
        self.strn = strn

    def __str__(self):
        return repr(self.strn)


class ArgumentError(Exception):

    def __init__(self, message, errors=None):

        # Call the base class constructor with the parameters it needs
        """

        :param message: The error message output
        :param errors: An optional dict of error messages

        """

        super(ArgumentError, self).__init__(message)

        self.errors = errors


class MapFile(object):
    """MapFile abstracts the atomic raster file read/write processes.
    """

    def __init__(self, filename, verbose=False):
        """
        :param filename: path to the raster file to be opened
        :param verbose: indicate whether we would like to print out certain metrics regarding the file opened
        """
        self.file = filename
        try:
            self.img = rasterio.open(self.file)
        except RuntimeError as e:
            print('Unable to open {}'.format(str(self.file)))
            print(e)
            sys.exit(1)
        self.crs_wkt = self.img.crs_wkt
        spheroid_start = self.crs_wkt.find("SPHEROID[") + len("SPHEROID")
        spheroid_end = self.crs_wkt.find("AUTHORITY", spheroid_start)
        self.spheroid = str(self.crs_wkt)[spheroid_start:spheroid_end]
        self.spheroid = self.spheroid.strip('[]').split(',')
        self.semimajor = float(self.spheroid[1])
        self.spheroid[2] = self.spheroid[2].strip(']]')
        self.inverse_flattening = float(self.spheroid[2])
        self.flattening = float(1 / self.inverse_flattening)
        self.semiminor = float(self.semimajor * (1 - self.flattening))
        self.eccentricity = math.sqrt(2 * self.flattening - self.flattening * self.flattening)
        self.geotransform = self.img.meta['transform']
        self.nodatavalue, self.data = None, None
        self.nodatavalue = self.img.meta['nodata']
        self.ncol = self.img.meta['width']
        self.nrow = self.img.meta['height']
        self.rotation = self.geotransform[2]  # rotation, 0 if image is 'north-up'
        self.originX = self.geotransform[0]  # top-left x
        self.originY = self.geotransform[3]  # top-left y
        self.pixelWidth = self.geotransform[1]  # w/e pixel resoluton
        self.pixelHeight = self.geotransform[5]  # n/s pixel resolution

        if verbose is True:
            print("GeoT:{}".format(self.geotransform))
            print("Metadata:{}".format(self.img.meta))


class RetrievePointException(Exception):
    """ReadExceptions occur in response to invalid queries on coordinates

    These invalid queries include coordinates that are not within the extent of the current raster,
    or those that are either not available or contain a recognized 'no-data' value.

    Note:
        I'm unsure how NaN values translate on the Matlab side. It is also
        possible to return the 'no-value' float directly. This value can typically be found in
        a raster files header, and are stored in the MapFile class' attribute `nodatavalue`

    Args:
        strn (str): Human readable string describing the exception.

    Attributes:
        strn (str): Human readable string describing the exception.

    """

    def __init__(self, strn):
        self.strn = strn

    def __str__(self):
        return repr(self.strn)


class Map(MapFile):
    """

    The Map Class offers both atomic and advanced map read operations

    The Map Class is built on top of the MapFile class for low-level file reading operations.
    Map depends on the GDAL and Rasterio abstraction layers.

    """

    def __init__(self, filename, verbose=False, **kwargs):
        super(Map, self).__init__(filename, verbose, **kwargs)
        self.file_name = filename
        self.adjacentElevations = np.zeros((3, 3))
        self.vehicles = {}
        self.ds = gdal.Open(self.file_name)

    def lat_lon_to_pixel(self, coords):
        """

        First open the file with gdal (see if we can get around this), then retrieve its geotransform.
        Next, obtain a spatial reference, and perform a coordinate transformation.

        Return the pixel pair corresponding to the input coordinates given in lat/lon
        :param coords: A named Tuple of type 'Coordinate' containing a lat/lon pair
        :return: A named tuple of type PixelPair containing an x/y pair

        """
        ds = self.ds
        gt = self.geotransform
        srs = osr.SpatialReference()
        srs.ImportFromWkt(ds.GetProjection())
        srs_lat_lon = srs.CloneGeogCS()
        ct = osr.CoordinateTransformation(srs_lat_lon, srs)
        (x, y, holder) = ct.TransformPoint(coords.lon, coords.lat)
        x = (x - gt[0]) / gt[1]
        y = (y - gt[3]) / gt[5]
        pixel_pair = PixelPair(x=int(x), y=int(y))
        return pixel_pair

    def get_point_elevation(self, coords):
        """
        Retrieve an elevation for a single Coordinate

        :param coordinate: Named tuple of type Coordinate containing a lat/lon pair
        :param mode: Indicates whether we are passing in a Coordinate of lat/lon or a PixelPair of x/y
        """

        lat = coords.lat
        lon = coords.lon
        pixel = None
        # @todo:figure out how to add exceptions in rasterio
        # gdal.UseExceptions() #so it doesn't print to screen everytime point is outside grid
        pixs = self.lat_lon_to_pixel(Coordinate(lat=lat, lon=lon))
        px = pixs.x
        py = pixs.y
        # print("pixel:")
        # print(px,py)
        try:  # in case raster isnt full extent
            # Window format is: ((row_start, row_stop), (col_start, col_stop))
            pixel = self.img.read(1, window=((py, py + 1), (px, px + 1)))

        except:
            RetrievePointException("Pixel read exception")

        return pixel[0][0]


class Coord(object):
    def __init__(self, data, prev, next):
        self.data = data  # data is a named tuple of type coordinate
        self.prev = prev  # prev is a reference to the coordinate before
        self.next = next  # reference to the next coordinate

    def __str__(self):
        return str(self.data)


def main():
    """
    params: script is called from MATLAB with string values that are converted to floats
    output: elevation point of the specified coordinate after it is translated into a pixel location
    """

    path = "C:/Users/bmluu/Desktop/USGS_Rasters/ned19_n37x50_w122x00_ca_santaclaraco_2006/ned19_n37x50_w122x00_ca_santaclaraco_2006.img"
    map = Map(path)

    # User args
    in_lat = float(sys.argv[1])
    in_lng = float(sys.argv[2])
    coor = Coordinate(lat=in_lat, lon=in_lng)

    height = map.get_point_elevation(coor)
    print(height)

    return

if __name__ == "__main__":
    main()
