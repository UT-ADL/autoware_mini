#!/usr/bin/env python

from lanelet2.io import Origin
from lanelet2.core import GPSPoint
from lanelet2.projection import UtmProjector


class WGS84ToUTMTransformer:
    def __init__(self, use_custom_origin, origin_lat=58.385345, origin_lon=26.726272):

        self.origin = Origin(origin_lat, origin_lon)
        if self.use_custom_origin:
            self.transformer = UtmProjector(self.origin, useOffset = True, throwInPaddingArea = False)
        else:
            self.transformer = UtmProjector(self.origin, useOffset = False, throwInPaddingArea = False)

    def transform_lat_lon(self, lat, lon, height):
        gps_point = GPSPoint(lat, lon, height)
        utm_point = self.transformer.forward(gps_point)
        # no need to return z, since it is not used in UTMProjector
        return utm_point.x, utm_point.y

    def correct_azimuth(self, lat, lon, azimuth):
        # TODO implement azimuth correction if necessary
        # https://gis.stackexchange.com/questions/115531/calculating-grid-convergence-true-north-to-grid-north
        return azimuth