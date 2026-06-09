# Formula 1 Quasi-Static Lap Time Simulator (LTS)

[![MATLAB](https://img.shields.io/badge/MATLAB-R2024b+-orange.svg)](https://www.mathworks.com/products/matlab.html)

A MATLAB-based **Quasi-Static Lap Time Simulator (Point-Mass Approach)** designed to analyze vehicle performance limits, aerodynamic downforce interactions, and tire traction boundaries (G-G Footprint) across iconic tracks like Monza and Monaco.

---


Here is the generated engineering telemetry for **Monza GP**, highlighting the distinct velocity profile and the extracted **G-G Diagram**:

<img width="926" height="513" alt="telem" src="https://github.com/user-attachments/assets/7a6ca0b8-7d4a-4b8e-a6c7-b601b0ee4b70" />

For a detailed technical report of the project, complete of mathematical formulas, look in the associated PDF file in the repository.

---

## Key Engineering Features
* **Aerodynamic-Driven Grip Modeling:** Computes continuous lateral speed limits ($v_{limit}$) by coupling mechanical tire friction ($\mu$) with speed-dependent aero downforce.
* **Dual-Pass Numerical Integration:** Features a *Forward Pass* (engine power and drag limited) and a *Backward Pass* (braking deceleration limited) to build the vehicle's dynamic velocity envelope.
* **G-G Diagram Generation:** Automatically maps the simulated longitudinal and lateral G-forces to evaluate vehicle utilization at its friction limits.

## How to Run the Simulation
1. Clone this repository: `git clone https://github.com/haroonaliarfat/f1-laptime-simulator.git`
2. Open MATLAB and navigate to the `src/` directory.
3. Run the Script `laptime_sim_grip.m`.
4. Select your preferred track from the interactive GUI pop-up.

## Model Evolution & Future Work
To enhance the fidelity from a point-mass approximation to a professional vehicle dynamics tier, the next iterations will feature:
1. **Friction Ellipse Integration:** Implementing combined tire slip constraints ($a_x$ and $a_y$ coupling).
2. **Transient Load Transfers:** Upgrading to a 3-DOF chassis model to capture pitch and roll vertical load sensitivities.
