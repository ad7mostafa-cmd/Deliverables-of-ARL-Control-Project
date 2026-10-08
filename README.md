## Name : Adham Mostafa Abdelmoez
### Mail : ad7mostafa@gmail.com 
### Phone Number : 01094572282

#### Full documentation and mathematical formulations are included in the pdf file.

## BENCHMARK Table

| Controller Mode | Best Lap Time (s) | Top Speed (m/s) | Mean CTE (m) | Max CTE (m) | RMS CTE (m) | Laps Completed / Status |
| :--- | :---: | :---: | :---: | :---: | :---: | :---: |
| Lateral PID (Reactive) | 78.86 | 7.5 | 0.394 | 2.75 | 0.558 | 3 |
| Pure Pursuit (Preview) | 69.4 | 8.1 | 0.030 | 0.031 | 0.053 | 3 |
| MPC (Optimal) | 62.4 | 7.2 | 0.072 | 0.51 | 0.101 | 3 |



Model Predictive Control (MPC) is the best option because it actively plans for the future instead of just reacting to the present, by choosing the path with the least cost considering car limits and constraints.


  PID Controller: This controller drives by looking only at the ground directly in front of the car. It only sees the immediate error in the current moment and changes the steering angle to fix it, which often causes the car to wobble or over-correct.

  Pure Pursuit: After determining the look ahead distance suitable with the current velocity of the car, the controller constantly draws arcs their center is the rear axle to intersect the path a point, which is the next target point for the car, but Pure Pursuit doesn't consider the car's limits and constraints.

  MPC: This controller drives like a professional human racer. It looks far ahead and uses a mathematical model to simulate the physics of the entire upcoming track. Before it even reaches a corner, it calculates the perfect combination of steering and gas to take the fastest path while safely obeying the car's actual mechanical limits.


## MPPI

Model Predictive Path Integral is an advanced control algorithm used to steer complex systems like robots and self-driving cars.
It belongs to the family Model Predictive Control (MPC).
Instead of trying to solve incredibly complex, exact mathematical equations to find the perfect route, MPPI uses a sampling-based approach.
It uses computing power and probability to "guess" thousands of possible futures, evaluate them, and combine them to find the best possible steering and throttling commands.

### HOW DOES IT WORK?

The algorithm operates on a continuous loop, looking slightly into the future. These are the steps:

  MPPI requires a mathematical or AI model to predict how the system behaves.

  1-Random Sampling: The controller generates thousands of random control sequences (like, random steering and throttle inputs) for the next few seconds.

  2-Simulation: It simulates where the system will end up for every single one of those random sequences over a short horizon.

  3-Cost Evaluation: Each simulated path is graded using a cost function. If a path hits an obstacle or deviates from the target, it gets a high penalty. If it stays safe and on target, it gets a     low cost.

  4-Path Integration: Instead of just picking the single best path, it takes a weighted average of all the simulated paths. The paths with the lowest costs are given the highest weights.
  This creates a smooth, highly optimized control signal .

  5-Execution: The system executes only the first step of this newly calculated optimal plan, and then repeats the entire process a fraction of a second later.


Because MPPI relies on simulating thousands of futures simultaneously, its implementation requires modern hardware and parallel processing.

   Hardware: MPPI is almost always implemented on Graphics Processing Units GPUs. GPUs are designed to handle thousands of simple math operations at the exact same time, making them perfect for       parallel sampling.

   Mathematics: We define a system state equation and a cost function S(x, u). The algorithm generates random noise from a Gaussian distribution to create the samples.

### Integration of MPPI with AI and Neural Networks 

  Integrating Artificial Intelligence and Deep Neural Networks with MPPI replaces rigid physics equations with learned dynamic models trained directly on real sensor data.
  In the future, this synergy unlocks truly adaptive control in high-stakes environments allowing autonomous vehicles, robotic systems to instantaneously adjust.


  
  

