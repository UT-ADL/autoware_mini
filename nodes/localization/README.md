# Localization Module Overview

The localization module in Autoware Mini is responsible for determining the precise position of the vehicle within a global coordinate frame. 

![Localization Architecture](/images/nodes/localization.png)

## Architecture

The localization module takes inputs from:
- **NovAtel GNSS Receiver**: Provides raw GNSS data (latitude, longitude, height) and undulation value for height correction calculation.

It produces outputs to:
- **Global and local planning modules**: Supplies the vehicle's current pose and velocity for path planning.
- **Control module**: Supplies the vehicle's current pose and velocity for planned trajectory following.

## Components

### GNSS Localization

The primary source of global positioning in Autoware Mini comes from the Novatel OEM7 GNSS receiver:

- **Novatel OEM7 Localizer**: Processes raw GNSS data (INSPVA and BESTPOS messages) and transforms WGS84 geographic coordinates into a local Cartesian coordinate system (UTM). This transformation enables seamless integration with local map data.

- **Novatel OEM7 Visualizer**: Provides real-time visualization of GNSS quality metrics, including solution status, number of satellites, position accuracy, and differential correction age. 

### Map Matching

Optional module that enhances localization accuracy beyond what GNSS alone can provide:

- **Lane Boundary Matcher**: Corrects GNSS position by matching comma.ai openpilot camera-detected lane boundaries with map-based lane boundaries. This vision-based correction compensates for GNSS drift.

## Data Flow

1. The GNSS receiver provides raw position data in WGS84 format (latitude, longitude, height)
2. The Novatel OEM7 Localizer transforms these coordinates into the local map frame (UTM)
3. (Optional) The Lane Boundary Matcher compares camera-detected lane markings with map data to refine the position
4. The corrected position is published as the vehicle's current pose
5. NovAtel OEM7 visualizer outputs visualization data for monitoring and debugging
