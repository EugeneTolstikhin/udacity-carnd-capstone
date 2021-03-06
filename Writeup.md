# Individual submission. Capstone project for Udacity's Self-Driving Car Nanodegree.

|        Name       |  Udacity Account Email  |
| :---------------- | ----------------------: |
| Ievgen Tolstikhin |  etolstikhin@gmail.com  |

## Description

This project is developed based on the simulator provided by Udacity. Due to Corona time, there is an official information that tit will not betested in the real car. For that reason the project is not adapted to the real car testing. The folowing modules are needed to be modified to be able to make the car drive and and react on the traffic lights:
1) Waypoint updater
2) Twist controller, which contains both twist controller and drive-by-wire node
3) Traffic light classification and detection

All other modules are not necessary to be modified to make the project work.

## Waypoint updater

This node subscribed to receive the current vehicle position, lane waypoints and traffic waypoints (current situation - it receives the next traffic signal waypoint) and publishes waypoints from the car's current position to the distance ahead. See [waypoint updater](./ros/src/waypoint_updater/waypoint_updater.py).

The following callback receives the traffic waypoint and keep it for calculation the distance to the goal:
```python
    def traffic_cb(self, msg):
        self.stopline_wp_idx = msg.data
```

The following callback receives the lane waypoints to be able to keep the lane during driving:
```python
    def waypoints_cb(self, waypoints):
        self.base_waypoints = waypoints
        if not self.waypoints_2d:
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)
```

The following callback receives the current vehicle position and keep it for calculation the distance to the goal:
```python
    def pose_cb(self, msg):
        self.pose = msg
```

When all the data is received the proper route (via waypoints) is generated considering if the traffic light is far away or not and if the decelerating is needed or not (the amount of wapoints considering in front of the car is kept inside ```LOOKAHEAD_WPS``` variable):
```python
    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_waypoints.waypoints[closest_idx : farthest_idx]

        if self.stopline_wp_idx == -1 or self.stopline_wp_idx >= farthest_idx:
            lane.waypoints = base_waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane
```

## Drive-by-wire Node (aka DBW) and Twist Controller

### Drive-By-Wire Node

This node represents a drive by wire controller. It receives current and requested steering/velocities, calculates throttle, brake and steering commands and publishes them to the vehicle.

### Twist Controller

This controller is responsible for acceleration and steering. The acceleration is controlled via PID controller. Steering is calculated using YawController which simply calculates needed angle to keep needed velocity.

See [drive-by-wire node](./ros/src/twist_controller/dbw_node.py) and [twist controller](./ros/src/twist_controller/twist_controller.py).

## Traffic light classification and detection

1) *Classification*. [This module](./ros/src/tl_detector/light_classification/tl_classifier.py) is skipped and it is used the traffic light data provided by Udacity (via ```styx_msgs/TrafficLight``` message). See [here](./ros/src/tl_detector/tl_detector.py):
    ```python
        def get_light_state(self, light):
            """Determines the current color of the traffic light

            Args:
                light (TrafficLight): light to classify

            Returns:
                int: ID of traffic light color (specified in styx_msgs/TrafficLight)

            """
            return light.state

            """
            if(not self.has_image):
                self.prev_light_loc = None
                return False

            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "bgr8")

            #Get classification
            return self.light_classifier.get_classification(cv_image)
            """
    ```
    The reason why the get_classification function is skipped that the time when the project is implemented the available computer has not much computentional power for Machine Learning part.

2) *Detection*. As the ahead waypoints' positions are known and the physical positions of the traffic lights are known as well then the task is splitted into 2 parts:

    a) **Calculate the distance between stop line and the vehicle.**
    ```python
    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color

        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        closest_light = None
        line_wp_idx = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if(self.pose):
            car_wp_idx = self.get_closest_waypoint(self.pose.pose.position.x, self.pose.pose.position.y)

            #TODO find the closest visible traffic light (if one exists)
            diff = len(self.waypoints.waypoints)
            for i, light in enumerate(self.lights):
                #Get stop line waypoint index
                line = stop_line_positions[i]
                temp_wp_idx = self.get_closest_waypoint(line[0], line[1])

                #Find closest stop line waypoint index
                d = temp_wp_idx - car_wp_idx
                if d >= 0 and d < diff:
                    diff = d
                    closest_light = light
                    line_wp_idx = temp_wp_idx

        if closest_light:
            state = self.get_light_state(closest_light)
            return line_wp_idx, state

        self.waypoints = None
        return -1, TrafficLight.UNKNOWN
    ```

    b) **Understand if the vehicle is allowed to move forward without decelerating and/or stop or not.**
    --> This means that if the light is ***RED*** then the car should start decelerating in advance and be ready to stop before the stop line. See [here](./ros/src/tl_detector/tl_detector.py):
    ```python
    def image_cb(self, msg):
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """

        self.has_image = True
        self.camera_image = msg

        if time.time() - self.last_time < .1:
            # Discard historical messages
            return

        light_wp, state = self.process_traffic_lights()
        self.last_time = time.time()

        '''
        Publish upcoming red lights at camera frequency.
        Each predicted state has to occur `STATE_COUNT_THRESHOLD` number
        of times till we start using it. Otherwise the previous stable state
        is used.
        '''
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1
    ```

    The explained part is not obvious from the code above. So how it works from the code perspective: The module ```Traffic light detector``` is subscribed to the message ```/image_color``` and manipulate with the received data inside the callback ```image_cb(self, msg)```. The nearest (the next one in the vehicle's route) traffic light's state is extracted to be able to understand what colour of traffic light is now. The proper waypoint or -1 is published to ```/traffic_waypoint``` message, which is read inside [waypoint_updater.py](./ros/src/waypoint_updater/waypoint_updater.py) where the further decision regarding keeping the speed or decelerating is triggered.
