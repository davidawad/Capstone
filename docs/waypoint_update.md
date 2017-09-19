# Waypoint Updater

## Smooth Deceleration
The deceleration speed is calculated by spline function, as defined in python scipy library.
```
from scipy import interpolate 
...
    def deceleration_waypoints(self, waypoints, stop_index, start_velocity):
        stop_distance = self.wp_distance(waypoints, 0, stop_index)
        # estimate distance per waypoint
        s0 = self.wp_distance(waypoints, 0, 1)

        x = np.array([-s0, 0, stop_distance, stop_distance+s0, stop_distance+s0*2])
        y = np.array([start_velocity, start_velocity, 0, 0, 0])
        tck = interpolate.splrep(x, y, s=0)
        xnew = 0
        for i in range(1, len(waypoints)):
            if ( i < stop_index):
                xnew += self.wp_distance(waypoints, i-1, i)
                ynew = interpolate.splev(xnew, tck, der=0)
                self.set_waypoint_velocity(waypoints, i, ynew)
            else:
                self.set_waypoint_velocity(waypoints, i, 0)
 
```
## Test Deceleration with rostopic with /test_notify 
My test repo change the tl_detector.py and add a topic '/test_notify' so that we can test the deceleration without traffic light detector implemented.  
When you see a traffic light, you can send 1 to /test_notify by
```
rostopic pub --once /test_notify std_msgs/Int32 1
```
This actually trigger some code to find the traffic light location and publish /traffic_waypoint.
The twist_cmd should show the velocity decreasing to zero before the traffic light.
Another command to publish /test_notify with 0 will trigger to publish the /traffic_waypoint with -1.
```
rostopic pub --once /test_notify std_msgs/Int32 0
```
The car will be able to drive again at cruise speed.

# MISC
## Server Hot Fix
For using VirtualBox with Windows 10 host, I need to make a following hotfix in order to let ROS communicate with simulator(running in Windows 10).  
**src/styx/server.py**
After import statments,
```python
# try to solve problem in Windows simulator
sio = socketio.Server(async_mode='eventlet')
eventlet.monkey_patch()
# sio = socketio.Server() #original statement
```
This is just for current environment I use(VirtualBox in Windows 10).  **Maybe it is not suitable for other developing environment.**
## Car Speed with Different Unit
Car cruise speed is set via the parameter(~velocity).  In waypoint_loader.py, self.velocity is set by parameter (~velocity).
```python
 def __init__(self):
        rospy.init_node('waypoint_loader', log_level=rospy.DEBUG)

        self.pub = rospy.Publisher('/base_waypoints', Lane, queue_size=1)

        self.velocity = rospy.get_param('~velocity')
        self.new_waypoint_loader(rospy.get_param('~path'))
        rospy.spin() 
```
And from the code in waypoint_loader.py, there is a conversion by multiplication 0.27778.
```python
 p.twist.twist.linear.x = float(self.velocity*0.27778)
```
where 0.27778 = 1000 / 60 * 60.  That means the conversion from KM/H to M/S.  
So the following is the value table for cruise speed of car


|          | Velocity Parameter(KM/H)                                                    | waypoint.twist.twist.linear.x (m/s) | Display in Dash MPH) |
|----------|-----------------------------------------------------------------------------|-------------------------------------|------------
|Simulator | 40 (km/h) defined in src/waypoint_loader/launch/waypoint_loader.launch      | 11.11 m/s                           |  24.85MPH
|Carla     | 10 (km/h) defined in src/waypoint_loader/launch/waypoint_loader_site.launch |   2.7 m/s                           |   6.21MPH


## Udacity Data Inconsistence
Yaw unit for waypoint_loader.py in 
Simultor: degree
Carla: radian
waypoint_loader.py load simulator waypoint yaw data from 'data/wp_yaw_const.txt'.  From the value range of data, it looks like the unit is degree.  But the 'churchlot_with_cars.csv' for Carla is obvious in radian.  We should check this again before submit.
