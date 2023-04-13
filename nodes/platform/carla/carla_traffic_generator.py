#!/usr/bin/env python

# Copyright (c) 2023 Autonomous Driving Lab (ADL), University of Tartu.
#
# This work is licensed under the terms of the MIT license.
# For a copy, see <https://opensource.org/licenses/MIT>.

"""Example node to generate traffic in the simulation. 
   Adapted from https://github.com/carla-simulator/carla/blob/master/PythonAPI/examples/generate_traffic.py
"""

import time
import rospy
import carla
import logging
from numpy import random
    

class CarlaTrafficGenerator(object):

    def __init__(self):
        
        #################
        ## Node params ##
        #################

        self.host = rospy.get_param('~host', default='127.0.0.1')
        self.port = rospy.get_param('~port', default=2000)
        self.tm_port = rospy.get_param('~tm_port', default=8000)

        # ------------------------------

        # Number of vehicles to spawn
        self.number_of_vehicles = rospy.get_param('~number_of_vehicles', default=30)
        # Vehicle filter
        self.filterv = rospy.get_param('~filterv', default='vehicle.*')
        # restrict to certain vehicle generation (values: "1","2","All" - default: "All")
        self.generationv = rospy.get_param('~generationv', default='All')

        # ------------------------------

        # Number of walkers to spawn
        self.number_of_walkers = rospy.get_param('~number_of_walkers', default=10)
        # Walker filter
        self.filterw = rospy.get_param('~filterw', default='walker.pedestrian.*')
        # restrict to certain pedestrian generation (values: "1","2","All" - default: "2")
        self.generationw = rospy.get_param('~generationw', default='All')

        # ------------------------------

        # Toggle car lights on/off
        self.car_lights_on = rospy.get_param('~car_lights_on', default=False)
        # Respawn dormant vehicles (vehicles that have been spawned but are not in the simulation)
        self.respawn_dormant_vehicles = rospy.get_param('~respawn_dormant_vehicles', default=False)

        # ------------------------------

        # Avoid spawning vehicles prone to accidents
        self.safe = rospy.get_param('~safe', default=False)
        # Activate asynchronous mode execution
        self.asynchronous_mode = rospy.get_param('~asynchronous_mode', default=True)
        # Activate hybrid mode for Traffic Manager
        self.hybrid_mode = rospy.get_param('~hybrid_mode', default=False)
        # Activate no rendering mode
        self.activate_no_rendering_mode = rospy.get_param('~activate_no_rendering_mode', default=False)

        ######################
        ## Class attributes ##
        ######################
        self.seed = None
        self.seed_walkers = 0
        self.seed_vehicles = 0


    def spawn_actors(self):

        logging.basicConfig(format='%(levelname)s: %(message)s', level=logging.INFO)

        vehicles_list = []
        walkers_list = []
        all_id = []
        client = carla.Client(self.host, self.port)
        client.set_timeout(10.0)
        synchronous_master = False
        random.seed(self.seed if self.seed is not None else int(time.time()))

        try:
            world = client.get_world()

            traffic_manager = client.get_trafficmanager(self.tm_port)
            traffic_manager.set_global_distance_to_leading_vehicle(2.5)
            if self.respawn_dormant_vehicles:
                traffic_manager.set_respawn_dormant_vehicles(True)
            if self.hybrid_mode:
                traffic_manager.set_hybrid_physics_mode(True)
                traffic_manager.set_hybrid_physics_radius(70.0)
            if self.seed is not None:
                traffic_manager.set_random_device_seed(self.seed)

            settings = world.get_settings()
            if not self.asynchronous_mode:
                traffic_manager.set_synchronous_mode(True)
                if not settings.synchronous_mode:
                    synchronous_master = True
                    settings.synchronous_mode = True
                    settings.fixed_delta_seconds = 0.05
                else:
                    synchronous_master = False
            else:
                print("You are currently in asynchronous mode. If this is a traffic simulation, \
                you could experience some issues. If it's not working correctly, switch to synchronous \
                mode by using traffic_manager.set_synchronous_mode(True)")

            if self.activate_no_rendering_mode:
                settings.no_rendering_mode = True
            world.apply_settings(settings)

            blueprints = self.get_actor_blueprints(world, self.filterv, self.generationv)
            blueprintsWalkers = self.get_actor_blueprints(world, self.filterw, self.generationw)

            if self.safe:
                blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
                blueprints = [x for x in blueprints if not x.id.endswith('microlino')]
                blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]
                blueprints = [x for x in blueprints if not x.id.endswith('cybertruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('t2')]
                blueprints = [x for x in blueprints if not x.id.endswith('sprinter')]
                blueprints = [x for x in blueprints if not x.id.endswith('firetruck')]
                blueprints = [x for x in blueprints if not x.id.endswith('ambulance')]

            blueprints = sorted(blueprints, key=lambda bp: bp.id)

            spawn_points = world.get_map().get_spawn_points()
            number_of_spawn_points = len(spawn_points)

            if self.number_of_vehicles < number_of_spawn_points:
                random.shuffle(spawn_points)
            elif self.number_of_vehicles > number_of_spawn_points:
                msg = 'requested %d vehicles, but could only find %d spawn points'
                logging.warning(msg, self.number_of_vehicles, number_of_spawn_points)
                self.number_of_vehicles = number_of_spawn_points

            # @todo cannot import these directly.
            SpawnActor = carla.command.SpawnActor
            SetAutopilot = carla.command.SetAutopilot
            FutureActor = carla.command.FutureActor

            # --------------
            # Spawn vehicles
            # --------------
            batch = []
            for n, transform in enumerate(spawn_points):
                if n >= self.number_of_vehicles:
                    break
                blueprint = random.choice(blueprints)
                if blueprint.has_attribute('color'):
                    color = random.choice(blueprint.get_attribute('color').recommended_values)
                    blueprint.set_attribute('color', color)
                if blueprint.has_attribute('driver_id'):
                    driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                    blueprint.set_attribute('driver_id', driver_id)

                blueprint.set_attribute('role_name', 'autopilot')

                # spawn the cars and set their autopilot and light state all together
                batch.append(SpawnActor(blueprint, transform)
                    .then(SetAutopilot(FutureActor, True, traffic_manager.get_port())))

            for response in client.apply_batch_sync(batch, synchronous_master):
                if response.error:
                    logging.error(response.error)
                else:
                    vehicles_list.append(response.actor_id)

            # Set automatic vehicle lights update if specified
            if self.car_lights_on:
                all_vehicle_actors = world.get_actors(vehicles_list)
                for actor in all_vehicle_actors:
                    traffic_manager.update_vehicle_lights(actor, True)

            # -------------
            # Spawn Walkers
            # -------------
            # some settings
            percentagePedestriansRunning = 0.0      # how many pedestrians will run
            percentagePedestriansCrossing = 0.0     # how many pedestrians will walk through the road
            if self.seed_walkers:
                world.set_pedestrians_seed(self.seed_walkers)
                random.seed(self.seed_walkers)
            # 1. take all the random locations to spawn
            spawn_points = []
            for i in range(self.number_of_walkers):
                spawn_point = carla.Transform()
                loc = world.get_random_location_from_navigation()
                if (loc != None):
                    spawn_point.location = loc
                    spawn_points.append(spawn_point)
            # 2. we spawn the walker object
            batch = []
            walker_speed = []
            for spawn_point in spawn_points:
                walker_bp = random.choice(blueprintsWalkers)
                # set as not invincible
                if walker_bp.has_attribute('is_invincible'):
                    walker_bp.set_attribute('is_invincible', 'false')
                # set the max speed
                if walker_bp.has_attribute('speed'):
                    if (random.random() > percentagePedestriansRunning):
                        # walking
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                    else:
                        # running
                        walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
                else:
                    print("Walker has no speed")
                    walker_speed.append(0.0)
                batch.append(SpawnActor(walker_bp, spawn_point))
            results = client.apply_batch_sync(batch, True)
            walker_speed2 = []
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    walkers_list.append({"id": results[i].actor_id})
                    walker_speed2.append(walker_speed[i])
            walker_speed = walker_speed2
            # 3. we spawn the walker controller
            batch = []
            walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
            for i in range(len(walkers_list)):
                batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
            results = client.apply_batch_sync(batch, True)
            for i in range(len(results)):
                if results[i].error:
                    logging.error(results[i].error)
                else:
                    walkers_list[i]["con"] = results[i].actor_id
            # 4. we put together the walkers and controllers id to get the objects from their id
            for i in range(len(walkers_list)):
                all_id.append(walkers_list[i]["con"])
                all_id.append(walkers_list[i]["id"])
            all_actors = world.get_actors(all_id)

            # wait for a tick to ensure client receives the last transform of the walkers we have just created
            if self.asynchronous_mode or not synchronous_master:
                world.wait_for_tick()
            else:
                world.tick()

            # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
            # set how many pedestrians can cross the road
            world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
            for i in range(0, len(all_id), 2):
                # start walker
                all_actors[i].start()
                # set walk to random point
                all_actors[i].go_to_location(world.get_random_location_from_navigation())
                # max speed
                all_actors[i].set_max_speed(float(walker_speed[int(i/2)]))

            print('spawned %d vehicles and %d walkers, press Ctrl+C to exit.' % (len(vehicles_list), len(walkers_list)))

            # Example of how to use Traffic Manager parameters
            traffic_manager.global_percentage_speed_difference(30.0)

            while True:
                if not self.asynchronous_mode and synchronous_master:
                    world.tick()
                else:
                    world.wait_for_tick()

        finally:

            if not self.asynchronous_mode and synchronous_master:
                settings = world.get_settings()
                settings.synchronous_mode = False
                settings.no_rendering_mode = False
                settings.fixed_delta_seconds = None
                world.apply_settings(settings)

            print('\ndestroying %d vehicles' % len(vehicles_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in vehicles_list])

            # stop walker controllers (list is [controller, actor, controller, actor ...])
            for i in range(0, len(all_id), 2):
                all_actors[i].stop()

            print('\ndestroying %d walkers' % len(walkers_list))
            client.apply_batch([carla.command.DestroyActor(x) for x in all_id])

            time.sleep(0.5)

    def get_actor_blueprints(self, world, filter, generation):
        bps = world.get_blueprint_library().filter(filter)

        if generation.lower() == "all":
            return bps

        # If the filter returns only one bp, we assume that this one needed
        # and therefore, we ignore the generation
        if len(bps) == 1:
            return bps

        try:
            int_generation = int(generation)
            # Check if generation is in available generations
            if int_generation in [1, 2]:
                bps = [x for x in bps if int(x.get_attribute('generation')) == int_generation]
                return bps
            else:
                print("   Warning! Actor Generation is not valid. No actor will be spawned.")
                return []
        except:
            print("   Warning! Actor Generation is not valid. No actor will be spawned.")
            return []

    def run(self):
        self.spawn_actors()

if __name__ == '__main__':
    rospy.init_node('carla_traffic_generator', log_level=rospy.INFO, anonymous=False)
    node = CarlaTrafficGenerator()
    node.run()

 