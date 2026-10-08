"""
High-Level Lateral Steering Controller: Geometric Pure Pursuit.
Calculates steering curvature from lookahead arc geometry.
"""

import math  # noqa: F401
import numpy as np  # noqa: F401


class PurePursuitController:
    """Adaptive Pure Pursuit lateral controller."""

    def __init__(self, wheelbase=1.25, kv=0.25, l_min=0.8, l_max=2.5,
                 max_steer_rad=math.radians(35.0)):
        self.L = wheelbase
        self.kv = kv
        self.l_min = l_min
        self.l_max = l_max
        self.max_steer_rad = max_steer_rad

    def compute_lookahead(self, v):
        """Adaptive lookahead distance: Ld = clip(kv * v + l_min, l_min, l_max)."""
        # TODO: Milestone 5.3 Step 1 — Adaptive Lookahead Horizon
        # The car looks further ahead at higher speeds to plan smoother turns.
        # Implement the speed-scaled lookahead formula and clamp it to the allowed range.
        ld = self.kv * abs(v) + self.l_min
        ld = np.clip (ld, self.l_min , self.l_max)

        return float (ld)

    def find_target_waypoint(self, x, y, path_points, lookahead):
        """Searches along path for the target waypoint at lookahead distance."""
        # TODO: Milestone 5.3 Step 2 — Target Waypoint Selection
        # This selects the goal point the car will steer toward.
        # Find the nearest waypoint on the path, then walk forward until
        # you reach one that is at least 'lookahead' meters away.
        all_distances = []

        for path_point in path_points :
            dist = math.hypot (path_point [0] - x, path_point[1]- y)   # calculating the distance using hypotenuse function. while path_point [0] is the x coordinate of the path point, and path_point[1] is the y coordinate of the path point.
            all_distances.append(dist)

        closest_point_index = all_distances.index(min(all_distances))   #Determining the index of the closest point to the path 

        total_track_points = len (path_points)

        for step in range (closest_point_index , closest_point_index + total_track_points):
            current_index = step % total_track_points   
            if all_distances [current_index] >= lookahead :  # We search for a point that its distance is equal to or greater than the lookahead to set it as the new target point
                target_point = path_points [current_index]
                return current_index , target_point


        

    def compute_steering(self, x, y, yaw, target_pt, lookahead):
        """Computes steering angle in radians using Pure Pursuit geometry."""
        # TODO: Milestone 5.3 Steps 3 & 4 — Coordinate Transformation & Arc Law
        # This is the core of Pure Pursuit: transform the target into the vehicle's
        # local frame, then use the arc geometry formula to compute the steering angle.
        
        # making the rear axle our new origin 
        dx = target_pt[0] - x 
        dy = target_pt[1] - y

        # Rotation of axes
        local_x = dx * math.cos (yaw) + dy * math.sin (yaw)
        local_y = - dx * math.sin (yaw) + dy * math.cos (yaw)

        # calculating the steering command 

        steering = math.atan( 2* self.L * local_y / lookahead **2 )
        steering = np.clip (steering , - self.max_steer_rad , self.max_steer_rad)

        return float (steering)