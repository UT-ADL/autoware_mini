import math
from pyproj import CRS, Transformer, Proj
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose

class SimulationToUTMTransformer:

    def __init__(self, use_custom_origin=True, origin_lat=58.385345, origin_lon=26.726272):
        
        """ Transforms coodinates from simulation coordinate system to UTM

        Parameters
        ----------
        use_custom_origin : bool
            Bool to set if custom offsetting should be applied to transformations
        origin_lat : init
            latitude offset to be used when use_custom_origin is true
        origin_long : init
            longitud offset to be used when use_custom_origin is true
        """

        sim_crs = Proj("+proj=tmerc +lat_0=58.382296 +lon_0=26.726196 +k=1 +x_0=0 +y_0=0 +datum=WGS84 +units=m +vunits=m +no_defs")
        self.utm_crs = Proj(CRS.from_epsg(25835))
        wgs84_crs = CRS.from_epsg(4326)

        wgs842utm_transformer = Transformer.from_proj(wgs84_crs, self.utm_crs)
        self.utm_origin_x, self.utm_origin_y = wgs842utm_transformer.transform(origin_lat, origin_lon)

        self.use_custom_origin = use_custom_origin

        self.sim2utm_transformer = Transformer.from_proj(sim_crs, self.utm_crs)
        self.utm2wgs84_transformer = Transformer.from_proj(self.utm_crs, wgs84_crs)

    def transform_pose(self, pose_sim):
        """ Transforms simulation pose into UTM coordinates

        Parameters
        ----------
        pose : geometry_msgs/Pose
            Pose to be transformed
        
        Returns
        -------
        geometry_msgs/Pose
            Transformed pose
        """
        pose_utm = Pose()
        
        pose_utm.position.x, pose_utm.position.y = \
            self.sim2utm_transformer.transform(pose_sim.position.x, pose_sim.position.y)

        lat, lon = self.utm2wgs84_transformer.transform(pose_utm.position.x, pose_utm.position.y)
        yaw_offset = self.utm_crs.get_factors(lon, lat).meridian_convergence
        yaw_offset = math.radians(yaw_offset)
        
        roll, pitch, yaw = euler_from_quaternion(
            [pose_sim.orientation.x, pose_sim.orientation.y, pose_sim.orientation.z, pose_sim.orientation.w])
        pose_utm.orientation.x, pose_utm.orientation.y, pose_utm.orientation.z, pose_utm.orientation.w = \
            quaternion_from_euler(roll, pitch, yaw + yaw_offset)

        if self.use_custom_origin:
            pose_utm.position.x -= self.utm_origin_x
            pose_utm.position.y -= self.utm_origin_y
        
        return pose_utm
