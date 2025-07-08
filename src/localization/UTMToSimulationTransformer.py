from pyproj import CRS, Transformer, Proj
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from geometry_msgs.msg import Pose

class UTMToSimulationTransformer:

    def __init__(self, use_custom_origin=True, origin_lat=58.385345, origin_lon=26.726272):
        
        """ Transforms coodinates from UTM to simulation coordinate system

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

        self.utm2sim_transformer = Transformer.from_proj(self.utm_crs, sim_crs)

    def transform_pose(self, pose_utm):
        """
        Transforms UTM pose into simulation coordinates
        """

        pose_sim = Pose()
        
        x = pose_utm.position.x + self.utm_origin_x
        y = pose_utm.position.y + self.utm_origin_y
        z = pose_utm.position.z

        pose_sim.position.x, pose_sim.position.y, pose_sim.position.z = self.utm2sim_transformer.transform(x, y, z)
        # 2D pose estimate is placed by hand the accuracy is quite low, hence don't see the need to correct the orientation
        pose_sim.orientation = pose_utm.orientation

        return pose_sim