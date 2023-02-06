#!/usr/bin/env python

import numpy as np
from pyproj import CRS, Transformer
from lanelet2.io import Origin
from lanelet2.core import GPSPoint
from lanelet2.projection import UtmProjector


class WGS84ToUTMTransformer:
    def __init__(self, use_custom_origin, origin_lat=58.385345, origin_lon=26.726272):

        self.origin = Origin(origin_lat, origin_lon)
        if self.use_custom_origin:
            self.transformer = UtmProjector(self.origin, True, False)
        else:
            self.transformer = UtmProjector(self.origin, False, False)

    def transform_lat_lon(self, lat, lon, height):
        gps_point = GPSPoint(lat, lon, height)
        utm_point = self.transformer.forward(gps_point)
        # no need to return z, since it is not used in UTMProjector
        return utm_point.x, utm_point.y

    def correct_azimuth(self, lat, lon, azimuth):
        # TODO implement azimuth correction if necessary
        return azimuth

class WGS84ToLest97Transformer:
    
    # https://epsg.io/3301
    # NB! axes: northing (origin_x), easting (origin_y)

    def __init__(self, use_custom_origin, origin_x=6465000, origin_y=650000):
        
        self.use_custom_origin = use_custom_origin
        self.origin_x = origin_x
        self.origin_y = origin_y

        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_lest97 = CRS.from_epsg(3301)
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_lest97)


        # TODO - replace with more general solution, like pyproj get_factors.meridian_convergence
        # Stuff necessary to calc meridian convergence
        # flattening of GRS-80  ellipsoid
        f = 1 / 298.257222101
        er = (2.0 * f) - (f * f)
        e = np.sqrt(er)

        # 2 standard parallels of lest_97
        B1 = 58.0 * (np.pi / 180)
        B2 = (59.0 + 20.0 / 60.0) * (np.pi / 180)

        # calculate constant
        m1 = np.cos(B1) / np.sqrt(1.0 - er * np.power(np.sin(B1), 2))
        m2 = np.cos(B2) / np.sqrt(1.0 - er * np.power(np.sin(B2), 2))
        
        t1 = np.sqrt(((1.0 - np.sin(B1)) / (1.0 + np.sin(B1))) * np.power((1.0 + e * np.sin(B1)) / (1.0 - e * np.sin(B1)), e))
        t2 = np.sqrt(((1.0 - np.sin(B2)) / (1.0 + np.sin(B2))) * np.power((1.0 + e * np.sin(B2)) / (1.0 - e * np.sin(B2)), e))

        self.convergence_const = (np.log(m1) - np.log(m2)) / (np.log(t1) - np.log(t2))
        self.refernece_meridian = 24.0

    def transform_lat_lon(self, lat, lon, height):
        coords = self.transformer.transform(lat, lon)
        
        # axes: northing, easting
        if self.use_custom_origin:
            northing = coords[0] - self.origin_x
            easting = coords[1] - self.origin_y
        else:
            northing = coords[0]
            easting = coords[1]

        # return fist easting, then northing to match x and y in ROS map frame
        return easting, northing

    def correct_azimuth(self, lat, lon, azimuth):
        # TODO - replace with more general solution, like pyproj get_factors.meridian_convergence
        # print("correction  : ", (self.convergence_const * (lon - self.refernece_meridian)))
        # print(Proj("EPSG:3301").get_factors(lat, lon))
        return azimuth - (self.convergence_const * (lon - self.refernece_meridian))