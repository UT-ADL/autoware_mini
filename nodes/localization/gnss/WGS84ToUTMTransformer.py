#!/usr/bin/env python

import math
from lanelet2.io import Origin
from lanelet2.core import GPSPoint
from lanelet2.projection import UtmProjector


class WGS84ToUTMTransformer:
    def __init__(self, use_custom_origin, origin_lat=58.385345, origin_lon=26.726272):

        self.origin = Origin(origin_lat, origin_lon)
        # find out zone number, multiply with width of zone (6 degrees) and add 3 degrees to get central meridian
        self.central_meridian = (origin_lon // 6.0) * 6.0 + 3.0

        # UtmProjector(Origin, useOffset, throwInPaddingArea)
        self.transformer = UtmProjector(self.origin, use_custom_origin, False)

    def transform_lat_lon(self, lat, lon, height):
        gps_point = GPSPoint(lat, lon, height)
        utm_point = self.transformer.forward(gps_point)
        # no need to return z, since it is not used in UTMProjector
        return utm_point.x, utm_point.y

    def correct_azimuth(self, lat, lon, azimuth):

        # calculate grid convergence and use to correct the azimuth
        # https://gis.stackexchange.com/questions/115531/calculating-grid-convergence-true-north-to-grid-north
        a = math.tan(math.radians(lon - self.central_meridian))
        b = math.sin(math.radians(lat))
        correction = math.degrees(math.atan(a * b))

        # TODO compare with CA = (Lambda - LambdaCM) * sin Theta

        return azimuth - correction