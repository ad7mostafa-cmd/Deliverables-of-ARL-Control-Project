"""
High-Level Lateral Steering Controller: Extended Kinematic Bicycle MPC.
Solves a constrained non-linear program over prediction horizon N using SciPy,
optimizing steering angle and longitudinal acceleration (mapped to throttle).
"""

import math  # noqa: F401
import numpy as np  # noqa: F401
from scipy.optimize import minimize  # noqa: F401


class KinematicBicycleMPC:
    """Nonlinear Model Predictive Control for an Extended Kinematic Bicycle Model.

    Optimizes future control sequences u = [delta_k, a_k] where steering angle delta_k
    and longitudinal acceleration a_k (mapped to throttle effort) are the control inputs,
    forward-simulating a 4-state extended kinematic bicycle model x = [x, y, theta, v]^T.
    """

    def __init__(self, wheelbase=1.25, dt=0.1, horizon=10,
                 max_steer_rad=math.radians(35.0), k_a=4.0,
                 max_accel=None, max_brake=None):
        self.L = wheelbase
        self.dt = dt
        self.N = horizon
        self.max_steer_rad = max_steer_rad
        self.k_a = float(max_accel if max_accel is not None else k_a)

        # Weights: heavily penalize lateral CTE, heading error, and steering rate
        self.w_lat = 30.0
        self.w_long = 1.0
        self.w_yaw = 10.0
        self.w_v = 10.0
        self.w_steer = 0.2
        self.w_dsteer = 12.0
        self.w_accel = 0.1
        

        self.last_u = np.zeros(2 * self.N)  # warm-start [delta_0, a_0, delta_1, a_1, ...]

    def solve(self, x0, ref_trajectory, current_steer=0.0):
        """Solves MPC optimization problem over horizon N.

        x0: [x, y, yaw, v]
        ref_trajectory: list of length N containing [x_ref, y_ref, yaw_ref, v_ref]
        current_steer: actual current steering angle in radians
        Returns: (steer_rad, throttle_cmd in [-1.0, 1.0])
        """
        # ======================================================================
        # TODO: Milestone 5.4 — Extended Kinematic Bicycle MPC
        #
        # 1. Horizon & Bounds Setup:
        #    - Determine effective horizon N = min(self.N, len(ref_trajectory)).
        #    - If N < 2, return (0.0, 0.0).
        #    - Construct variable bounds for the decision vector:
        #      u = [delta_0, a_0, delta_1, a_1, ..., delta_N-1, a_N-1]
        #      where delta_k in [-self.max_steer_rad, self.max_steer_rad] (steering input)
        #      and a_k in [-self.k_a, self.k_a] (longitudinal acceleration input).

        N = min(self.N, len(ref_trajectory))      
    
        if N < 2:
            return (0.0, 0.0)
            
        bounds = []
        for _ in range(N):           
            bounds.append((-self.max_steer_rad, self.max_steer_rad))
            bounds.append((-self.k_a, self.k_a))
        #
        # 2. Objective Function objective(u):
        #    - Unpack state [x, y, yaw, v] from x0 and set prev_delta = current_steer.
        #    - For each horizon step k in 0 .. N-1:
        #        a. Forward simulate state using discrete Extended Kinematic Bicycle equations
        #           (where longitudinal velocity v is an explicit state variable integrated
        #           forward with acceleration input a_k)
        #        b. Project tracking error into the path-aligned Frenet frame.
        #        c. Accumulate weighted quadratic costs:
        #           lateral CTE, heading error, speed error, steering, slew rate, accel.
        #        d. Update prev_delta = delta_k.
        #    - Return total cost.

        def objective (u):
            cost = 0.0 

            # unpacking 
            x , y , yaw , v = x0
            prev_delta = current_steer 

           

            for k in range (N):
                 # Extract the current guess for steering and a from the decision vector 
                delta_k = u[2 * k]
                a_k = u[2*k +1]

                x+= v * math.cos(yaw) * self.dt 
                y+=v * math.sin(yaw) * self.dt 
                yaw+= (v / self.L) * math.tan(delta_k) * self.dt 
                v+= a_k * self.dt 

                x_ref, y_ref, yaw_ref, v_ref = ref_trajectory[k]
                dx= x-x_ref
                dy= y-y_ref 
                cte = -dx * math.sin(yaw_ref) + dy * math.cos(yaw_ref)

                yaw_err = yaw - yaw_ref
                if yaw_err > math.pi:
                    yaw_err -= 2 * math.pi
                elif yaw_err < -math.pi:
                    yaw_err += 2 * math.pi

                v_err = v - v_ref
                d_steer = delta_k - prev_delta 

                cost += (self.w_lat * cte**2 + self.w_yaw * yaw_err**2 + self.w_v * v_err **2 + self.w_steer * delta_k**2 + self.w_dsteer * d_steer**2 + self.w_accel * a_k**2) 

                prev_delta = delta_k 
            
            return cost



        




        #
        # 3. Warm-Start Initialization:
        #    - Construct u_init by shifting self.last_u forward by 1 time step.
        u_init = np.zeros(2 * N)
        if len(self.last_u) >= 2 * N:
            u_init[:-2] = self.last_u[2:2*N]
            
            u_init[-2:] = self.last_u[2*N-2:2*N]
        # 4. Numerical Optimization & Control Extraction:
        #    - Call scipy.optimize.minimize(objective, u_init, bounds=bounds,
        #                                   method='SLSQP',
        #                                   options={'maxiter': 25, 'ftol': 1e-3}).
        #    - Save optimal solution in self.last_u.
        #    - Extract first control step: delta_cmd = u*[0], accel_cmd = u*[1].
        #    - Map optimal acceleration a_0* to normalized throttle in [-1.0, 1.0]:
        #      throttle_cmd = accel_cmd / self.k_a
        #    - Return tuple: (delta_cmd, throttle_cmd).
        # ======================================================================
        optimal_solution = minimize (objective, u_init, bounds=bounds , method = 'SLSQP', options={'maxiter': 25, 'ftol': 1e-3} )

        self.last_u = optimal_solution.x

        delta_cmd = optimal_solution.x[0]
        accel_cmd = optimal_solution.x[1]

        throttle_cmd = accel_cmd / self.k_a
        throttle_cmd = max(-1.0, min(1.0, throttle_cmd))

        return float(delta_cmd), float(throttle_cmd)


