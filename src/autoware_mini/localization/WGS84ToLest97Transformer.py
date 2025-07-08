#!/usr/bin/env python3

import math
from pyproj import CRS, Transformer, Proj

class WGS84ToLest97Transformer:
    
    # https://epsg.io/3301
    # NB! axes: northing (origin_x), easting (origin_y)

    def __init__(self, use_custom_origin, origin_x, origin_y):
        
        self.use_custom_origin = use_custom_origin
        self.origin_x = origin_x
        self.origin_y = origin_y

        # for Lest97 coordinate system
        self.reference_meridian = 24.0

        self.crs_wgs84 = CRS.from_epsg(4326)
        self.crs_lest97 = CRS.from_epsg(3301)
        self.transformer = Transformer.from_crs(self.crs_wgs84, self.crs_lest97)

        # Similar solution: https://gis.stackexchange.com/questions/116823/calculating-grid-convergence-for-lambert-conformal-conic

        # Stuff necessary to calc meridian convergence
        # flattening of GRS-80  ellipsoid
        f = 1 / 298.257222101
        er = (2.0 * f) - (f * f)
        e = math.sqrt(er)

        # 2 standard parallels of lest_97
        B1 = 58.0 * (math.pi / 180)
        B2 = (59.0 + 20.0 / 60.0) * (math.pi / 180)

        # calculate constant
        m1 = math.cos(B1) / math.sqrt(1.0 - er * (math.sin(B1) ** 2))
        m2 = math.cos(B2) / math.sqrt(1.0 - er * (math.sin(B2) ** 2))
        
        t1 = math.sqrt(((1.0 - math.sin(B1)) / (1.0 + math.sin(B1))) * ((1.0 + e * math.sin(B1)) / (1.0 - e * math.sin(B1))) ** e)
        t2 = math.sqrt(((1.0 - math.sin(B2)) / (1.0 + math.sin(B2))) * ((1.0 + e * math.sin(B2)) / (1.0 - e * math.sin(B2))) ** e)

        self.convergence_const = (math.log(m1) - math.log(m2)) / (math.log(t1) - math.log(t2))


    def transform_lat_lon(self, lat, lon, height):
        coords = self.transformer.transform(lat, lon)
        
        # axes: northing, easting
        if self.use_custom_origin:
            northing = coords[0] - self.origin_x
            easting = coords[1] - self.origin_y
        else:
            northing = coords[0]
            easting = coords[1]

        # return frist easting, then northing to match x and y in ROS map frame
        return easting, northing

    def correct_azimuth(self, lat, lon, azimuth):
        # TODO - replace with more general solution, like pyproj get_factors.meridian_convergence
        # print("correction_long : %f" % (self.convergence_const * (lon - self.reference_meridian)))
        
        # calculate grid convergence - problem might be that lest97 has cone as projection surface
        #a = math.tan(math.radians(lon - 24.0))
        #b = math.sin(math.radians(lat))
        #correction = math.degrees(math.atan(a * b))
        #print("correction_utm  : %f" % correction)

        # calculate meridian convergence
        # print("correction_getf : %f" % Proj(self.crs_lest97).get_factors(lon, lat).meridian_convergence)

        return azimuth - (self.convergence_const * (lon - self.reference_meridian))