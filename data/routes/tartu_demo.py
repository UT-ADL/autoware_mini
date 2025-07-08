"""
Import file for all Tartu demo scenarios.
This file imports scenario classes from scenarios folder to make them available to scenario_runner.
"""

import os
import sys

# Add the scenarios directory to the path so we can import directly
scenarios_dir = os.path.join(os.path.dirname(__file__), 'scenarios')
if scenarios_dir not in sys.path:
    sys.path.insert(0, scenarios_dir)

# Now we can import directly from the scenarios
from bus_from_the_right_1 import *
from bus_from_the_right_2 import *
from LeftTurnPedestrianCrossing import *
from LeftTurnPedestrianCrossing2 import *
from PedestrianCrossingKaubamaja1 import *
from PedestrianCrossingKaubamaja2 import *
from PedestrianCrossingKaubamaja3 import *
from RightTurnPedestrianCrossing1 import *
from RightTurnPedestrianCrossing2 import *
from RightTurnPedestrianCrossing3 import *
from roller_from_the_right_1 import *
from roller_from_the_right_2 import *
from roller_from_the_right_3 import *
from StandingPedestrian import *
from StandingPedestrianCrossing import *
from StandingPedestrianCrossing2 import *
from StandingPedestrianCrossing3 import *
from StandingPedestrianCrossing4 import *
from traffic_light_controller import *
from vehicle_from_the_left_1 import *
from vehicle_from_the_left_2 import *
from vehicle_from_the_left_3 import *
from vehicle_from_the_left_4 import *
from vehicle_roundabout_1 import *
from vehicle_roundabout_2 import *
from vehicle_roundabout_3 import *
from vehicle_roundabout_4 import *
from WalkingPedestrianCrossing import *
from WalkingPedestrianCrossing2 import *

# Add any other scenario imports here as needed

# Print confirmation
print("Tartu demo lap scenarios loaded successfully")
