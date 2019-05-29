# Mavros-AUTO.MISSION-mode
This script will activate AUTO.MISSION mode along with pushing waypoints to be followed by the drone while PX4 gazebo simulation wrapped with mavros running  for VTOL quad-plane.

## AUTO_MISSION:-

The aircraft obeys the programmed mission sent by the ground control station (GCS). If no mission received, aircraft will LOITER at current position instead.
OFFBOARD (Offboard) In this mode the position, velocity or attitude reference / target / setpoint is provided by a companion computer connected via serial cable and MAVLink. The offboard setpoint can be provided by APIs like MAVROS or Dronekit. (reference - https://dev.px4.io/en/concept/flight_modes.html)
### Prerequisites
You must have things installed from here https://github.com/Mohit505Git/MAVROS-PX4-Gazebo-setup 
and if you have everything installed then we are good to go!

The script is well commented, so most of the things are explained there. However, I will mention few important points
1) Along with  the script, gazebo, mavros wrapper, you will also need Q Ground Controller running in the background.
1.1) Q Ground Controller can be downloaded from here (http://qgroundcontrol.com/).

## Running the tests
 run normally as a python script.
```
user@Desktop:~/yourDir$ python wayPointMission.py 
```

### Script in detail

 If you want to change the starting index of the waypoint then make chnages here.

```
wpPushService(start_index=0,waypoints=wps)#start_index = the index at which we want the mission to start
```

## Change waypoints from here.

```
w = wayp0.setWaypoints(3,84,True,True,0.0,0.0,0.0,float('nan'),47.397713,8.547605,50)
```
## Similar implimentation can be found at (https://gist.github.com/shakthi-prashanth-m/cb70406b8786b80a17b520cce663f3b8)
## Authors

* **Mohit Singh**
