import numpy as np
import matplotlib.pyplot as plt
from coast_table import VEL_RANGE, CD_RANGE, ALTITUDE_TABLE


def get_remaining_altitude(velocity, cd):
    """Get remaining altitude to apogee using bilinear interpolation."""
    vel = np.clip(velocity, VEL_RANGE[0], VEL_RANGE[-1])
    cd_val = np.clip(cd, CD_RANGE[0], CD_RANGE[-1])

    vel_idx = np.searchsorted(VEL_RANGE, vel) - 1
    vel_idx = np.clip(vel_idx, 0, len(VEL_RANGE) - 2)
    cd_idx = np.searchsorted(CD_RANGE, cd_val) - 1
    cd_idx = np.clip(cd_idx, 0, len(CD_RANGE) - 2)

    v0, v1 = VEL_RANGE[vel_idx], VEL_RANGE[vel_idx + 1]
    c0, c1 = CD_RANGE[cd_idx], CD_RANGE[cd_idx + 1]

    vel_weight = (vel - v0) / (v1 - v0) if v1 != v0 else 0.0
    cd_weight = (cd_val - c0) / (c1 - c0) if c1 != c0 else 0.0

    alt_00 = ALTITUDE_TABLE[vel_idx, cd_idx]
    alt_01 = ALTITUDE_TABLE[vel_idx, cd_idx + 1]
    alt_10 = ALTITUDE_TABLE[vel_idx + 1, cd_idx]
    alt_11 = ALTITUDE_TABLE[vel_idx + 1, cd_idx + 1]

    alt_0 = alt_00 * (1 - vel_weight) + alt_10 * vel_weight
    alt_1 = alt_01 * (1 - vel_weight) + alt_11 * vel_weight
    return alt_0 * (1 - cd_weight) + alt_1 * cd_weight

# Constants from config.h
MASS = 0.603  # kg
BASE_CD = 0.44  # From OpenRocket
AREA = (np.pi * (6.6 / 100.0) ** 2) / 4.0  # m^2
RHO = 1.225  # kg/m^3
G = 9.80665  # m/s^2
rhoA = RHO * AREA

# Simulation parameters
DT = 0.0005  # 2 kHz physics simulation
# Flight computer (control) and estimator timings
FC_DT = 0.0005   # 2 kHz flight computer
EST_DT = 0.002   # 500 Hz Cd estimator
MAX_MOTOR_VEL = 60.0  # units/s (0-24 in 0.4s)
MOTOR_MIN = 0.0
MOTOR_MAX = 22.0

# Cd relationship: linear from BASE_CD at position 0 to 4.0 at position 24
CD_MIN = BASE_CD
CD_MAX = 4.0

# Cd estimator parameters (from estimator.cpp)
ALPHA_CD = 0.05  # Low-pass filter coefficient at 500 Hz
CONTROL_VEL_START = 55.0  # m/s, velocity threshold for deploying airbrakes
TARGET_ALTITUDE = 228.6  # m, target apogee altitude
TARGET_CD_MIN = float(CD_RANGE[0])
TARGET_CD_MAX = float(CD_RANGE[-1])

# Cd controller tuning (match firmware-style closed-loop Cd tracking)
CD_CTRL_KP = 5.0  # position units per Cd error
CD_CTRL_KI = 25.0  # position units per (Cd error * s)
CD_CTRL_ERR_DEADBAND = 0.01  # Cd deadband to reduce chatter


class CdPiController:
    """PI controller that drives motor position to track target Cd."""

    def __init__(self, kp, ki, deadband, motor_min, motor_max):
        self.kp = kp
        self.ki = ki
        self.deadband = deadband
        self.motor_min = motor_min
        self.motor_max = motor_max
        self.integral = motor_min
        self.active = False

    def reset(self, current_position):
        self.integral = float(np.clip(current_position, self.motor_min, self.motor_max))
        self.active = False

    def update(self, target_cd, measured_cd, current_position, dt, enabled=True):
        if not enabled:
            self.reset(current_position)
            return self.motor_min

        if not self.active:
            self.active = True
            self.integral = float(np.clip(current_position, self.motor_min, self.motor_max))

        dt = float(np.clip(dt, 1e-4, 0.05))
        error = target_cd - measured_cd
        if abs(error) < self.deadband:
            error = 0.0

        unsat_target = self.integral + self.kp * error
        target = float(np.clip(unsat_target, self.motor_min, self.motor_max))

        # Anti-windup: do not integrate if saturated and error pushes further out.
        pushing_high = target >= self.motor_max and error > 0.0
        pushing_low = target <= self.motor_min and error < 0.0
        if not (pushing_high or pushing_low):
            self.integral += self.ki * error * dt
            self.integral = float(np.clip(self.integral, self.motor_min, self.motor_max))

        target = self.integral + self.kp * error
        return float(np.clip(target, self.motor_min, self.motor_max))


class ThrustCurve:
    """Simple linear interpolation for thrust curve."""
    def __init__(self, times, thrusts):
        self.times = times
        self.thrusts = thrusts
        
    def __call__(self, t):
        """Interpolate thrust at time t."""
        if t <= self.times[0]:
            return self.thrusts[0]
        if t >= self.times[-1]:
            return 0.0
        
        # Find bracketing indices
        idx = np.searchsorted(self.times, t)
        if idx >= len(self.times):
            return 0.0
        
        # Linear interpolation
        t0, t1 = self.times[idx-1], self.times[idx]
        f0, f1 = self.thrusts[idx-1], self.thrusts[idx]
        
        if t1 == t0:
            return f0
        
        return f0 + (f1 - f0) * (t - t0) / (t1 - t0)


def load_thrust_curve(filename):
    """Load thrust curve from CSV file."""
    times = []
    thrusts = []
    
    with open(filename, 'r') as f:
        # Skip header lines
        for _ in range(5):
            f.readline()
        
        # Read data
        for line in f:
            if line.strip():
                parts = line.strip().split(',')
                if len(parts) == 2:
                    try:
                        times.append(float(parts[0]))
                        thrusts.append(float(parts[1]))
                    except ValueError:
                        pass
    
    return ThrustCurve(np.array(times), np.array(thrusts))


def get_cd_from_position(position):
    """Calculate Cd based on airbrake position (0-24)."""
    # Linear interpolation between CD_MIN and CD_MAX
    return CD_MIN + (CD_MAX - CD_MIN) * (position / MOTOR_MAX)


def rocket_derivatives(t, state, thrust_func, motor_position):
    """
    Calculate derivatives for rocket equations of motion.
    state = [position, velocity]
    Returns [velocity, acceleration]
    """
    pos, vel = state
    
    # Forces
    thrust = thrust_func(t)
    drag_cd = get_cd_from_position(motor_position)
    drag = -0.5 * RHO * AREA * drag_cd * vel * abs(vel)  # Sign correct for direction
    weight = -MASS * G
    
    # Acceleration
    accel = (thrust + drag + weight) / MASS
    
    return np.array([vel, accel])


def rk4_step(t, state, dt, thrust_func, motor_position):
    """Perform one RK4 integration step."""
    k1 = rocket_derivatives(t, state, thrust_func, motor_position)
    k2 = rocket_derivatives(t + dt/2, state + dt*k1/2, thrust_func, motor_position)
    k3 = rocket_derivatives(t + dt/2, state + dt*k2/2, thrust_func, motor_position)
    k4 = rocket_derivatives(t + dt, state + dt*k3, thrust_func, motor_position)
    
    return state + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)


def update_motor_position(current_pos, target_pos, dt):
    """Update motor position with velocity limiting."""
    max_change = MAX_MOTOR_VEL * dt
    delta = target_pos - current_pos
    
    if abs(delta) <= max_change:
        return target_pos
    else:
        return current_pos + np.sign(delta) * max_change


def update_cd_estimate(cd_estimate, accel, vel):
    """
    Update Cd estimate using low-pass filter (from estimator.cpp).
    Only updates when accel < -G (deceleration phase).
    Runs at 500 Hz.
    """
    if accel < -G and abs(vel) > 0.1:  # Avoid division by zero
        # Current Cd from force balance: drag = 0.5 * rho * A * Cd * v^2
        # accel = (drag - weight) / mass
        # So: Cd = -(2 * mass * (accel + G)) / (rho * A * v * |v|)
        curr_cd = -(2.0 * MASS * (accel + G)) / (rhoA * vel * abs(vel))
        # Low-pass filter at 500 Hz
        cd_estimate = (1.0 - ALPHA_CD) * cd_estimate + ALPHA_CD * curr_cd
    
    return cd_estimate


def find_target_cd(pos, vel, _cd_estimate):
    """Solve target Cd continuously using bisection on bilinear lookup table."""
    pred_min = pos + get_remaining_altitude(vel, TARGET_CD_MIN)
    pred_max = pos + get_remaining_altitude(vel, TARGET_CD_MAX)

    # Same saturation behavior as firmware logic.
    if TARGET_ALTITUDE >= pred_min:
        return TARGET_CD_MIN
    if TARGET_ALTITUDE <= pred_max:
        return TARGET_CD_MAX

    lo = TARGET_CD_MIN
    hi = TARGET_CD_MAX
    for _ in range(14):
        mid = 0.5 * (lo + hi)
        pred = pos + get_remaining_altitude(vel, mid)
        if pred > TARGET_ALTITUDE:
            lo = mid
        else:
            hi = mid

    return 0.5 * (lo + hi)


def flight_computer(pos, vel, accel, cd_estimate, motor_position, cd_controller, dt):
    """
    Flight computer algorithm with optimal control.
    Uses coast phase lookup table to find optimal Cd, then calculates
    required motor position based on current drag characteristics.
    
    Returns target motor position and target Cd.
    """
    # Only activate control during coast phase (match firmware):
    # stay closed during any residual positive accel (late motor burn)
    if accel >= 0:
        cd_controller.reset(MOTOR_MIN)
        return MOTOR_MIN, np.nan

    target_cd = find_target_cd(pos, vel, cd_estimate)
    target_position = cd_controller.update(target_cd, cd_estimate, motor_position, dt)
    return target_position, target_cd


def run_simulation():
    """Run the rocket simulation."""
    # Load thrust curve
    thrust_func = load_thrust_curve('AeroTech_F35W.csv')
    
    # Initialize state
    t = 0.0
    state = np.array([0.0, 0.0])  # [position, velocity]
    motor_position = 0.0
    target_motor_position = 0.0
    cd_estimate = BASE_CD
    target_cd = np.nan

    cd_controller = CdPiController(
        CD_CTRL_KP, CD_CTRL_KI, CD_CTRL_ERR_DEADBAND, MOTOR_MIN, MOTOR_MAX
    )
    
    # Storage for plotting
    times = []
    positions = []
    velocities = []
    accelerations = []
    motor_positions = []
    target_motor_positions = []
    cd_estimates = []
    target_cds = []
    airbrake_forces = []
    predicted_apogees = []
    
    # Flight computer and estimator timing
    last_fc_time = 0.0
    last_est_time = 0.0
    
    # Run simulation until apogee
    prev_vel = 0.0
    step = 0
    while True:
        # Store current state
        times.append(t)
        positions.append(state[0])
        velocities.append(state[1])
        
        # Calculate acceleration for this timestep
        thrust = thrust_func(t)
        drag_cd = get_cd_from_position(motor_position)
        drag = -0.5 * RHO * AREA * drag_cd * state[1] * abs(state[1])
        weight = -MASS * G
        accel = (thrust + drag + weight) / MASS
        accelerations.append(accel)
        
        # Calculate airbrake force (additional drag beyond base Cd)
        airbrake_force = 0.5 * RHO * AREA * (drag_cd - BASE_CD) * state[1] * abs(state[1])
        airbrake_forces.append(airbrake_force)
        
        # Update estimator at EST_DT
        if t - last_est_time >= EST_DT - 1e-12:
            cd_estimate = update_cd_estimate(cd_estimate, accel, state[1])
            last_est_time = t

        # Run flight computer at FC_DT (2 kHz) and gate by velocity threshold
        if t - last_fc_time >= FC_DT - 1e-12:
            if state[1] <= CONTROL_VEL_START:
                target_motor_position, target_cd = flight_computer(
                    state[0], state[1], accel, cd_estimate, motor_position, cd_controller, FC_DT
                )
            else:
                cd_controller.reset(MOTOR_MIN)
                target_motor_position = MOTOR_MIN
                target_cd = np.nan
            last_fc_time = t
        
        # Calculate predicted apogee only during coast (deceleration)
        if accel <= 0.0:
            remaining_alt = get_remaining_altitude(state[1], cd_estimate)
            predicted_apogee = state[0] + remaining_alt
        else:
            predicted_apogee = np.nan
        predicted_apogees.append(predicted_apogee)
        
        cd_estimates.append(cd_estimate)
        target_motor_positions.append(target_motor_position)
        target_cds.append(target_cd)
        
        # Update motor position with rate limiting (runs at physics rate, DT)
        # Update motor position with rate limiting
        motor_position = update_motor_position(motor_position, target_motor_position, DT)
        motor_positions.append(motor_position)
        
        # RK4 step
        state = rk4_step(t, state, DT, thrust_func, motor_position)
        
        # Check for apogee (velocity changes from positive to negative)
        if state[1] <= 0 and prev_vel > 0 and t > 0.1:
            print(f"Apogee reached at t={t:.3f}s, altitude={state[0]:.2f}m")
            break
        
        prev_vel = state[1]
        t += DT
        step += 1
        
        # Safety check
        if t > 20.0:
            print(f"Simulation timeout at t={t:.3f}s")
            break
    
    return (np.array(times), np.array(positions), np.array(velocities), 
            np.array(accelerations), np.array(motor_positions), 
            np.array(target_motor_positions), np.array(cd_estimates),
            np.array(target_cds), np.array(airbrake_forces), np.array(predicted_apogees))


def plot_results(times, positions, velocities, accelerations, 
                motor_positions, target_motor_positions, cd_estimates,
                target_cds, airbrake_forces, predicted_apogees):
    """Create interactive plots of simulation results."""
    fig, axes = plt.subplots(3, 2, figsize=(14, 10))
    fig.suptitle('Rocket Flight Simulation Results', fontsize=16, fontweight='bold')
    
    # Position with predicted apogee
    axes[0, 0].plot(times, positions, 'b-', linewidth=2, label='Actual Altitude')
    axes[0, 0].plot(times, predicted_apogees, 'g--', linewidth=1.5, alpha=0.7, label='Predicted Apogee')
    axes[0, 0].axhline(y=TARGET_ALTITUDE, color='red', linestyle=':', alpha=0.5, label='Target Altitude')
    axes[0, 0].set_xlabel('Time (s)', fontsize=11)
    axes[0, 0].set_ylabel('Position (m)', fontsize=11)
    axes[0, 0].set_title('Altitude vs Time', fontweight='bold')
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend()
    
    # Velocity
    axes[0, 1].plot(times, velocities, 'r-', linewidth=2)
    axes[0, 1].axhline(y=CONTROL_VEL_START, color='gray', linestyle='--', alpha=0.5, label='Control threshold')
    axes[0, 1].set_xlabel('Time (s)', fontsize=11)
    axes[0, 1].set_ylabel('Velocity (m/s)', fontsize=11)
    axes[0, 1].set_title('Velocity vs Time', fontweight='bold')
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend()
    
    # Acceleration
    axes[1, 0].plot(times, accelerations / G, 'g-', linewidth=2)
    axes[1, 0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[1, 0].set_xlabel('Time (s)', fontsize=11)
    axes[1, 0].set_ylabel('Acceleration (G)', fontsize=11)
    axes[1, 0].set_title('Acceleration vs Time', fontweight='bold')
    axes[1, 0].grid(True, alpha=0.3)
    
    # Motor position (actual and target)
    axes[1, 1].plot(times, motor_positions, 'b-', linewidth=2, label='Actual Position')
    axes[1, 1].plot(times, target_motor_positions, 'r--', linewidth=2, alpha=0.7, label='Target Position')
    axes[1, 1].set_xlabel('Time (s)', fontsize=11)
    axes[1, 1].set_ylabel('Motor Position', fontsize=11)
    axes[1, 1].set_title('Airbrake Motor Position vs Time', fontweight='bold')
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend()
    axes[1, 1].set_ylim(-1, 26)
    
    # Cd estimate
    axes[2, 0].plot(times, cd_estimates, 'm-', linewidth=2)
    axes[2, 0].plot(times, target_cds, color='orange', linestyle='--', linewidth=1.8, alpha=0.8, label='Target Cd')
    axes[2, 0].axhline(y=BASE_CD, color='gray', linestyle='--', alpha=0.5, label='Base Cd')
    axes[2, 0].set_xlabel('Time (s)', fontsize=11)
    axes[2, 0].set_ylabel('Estimated Cd', fontsize=11)
    axes[2, 0].set_title('Drag Coefficient Estimate vs Time', fontweight='bold')
    axes[2, 0].grid(True, alpha=0.3)
    axes[2, 0].legend()
    
    # Airbrake force (excluding base drag)
    axes[2, 1].plot(times, airbrake_forces, 'c-', linewidth=2)
    axes[2, 1].set_xlabel('Time (s)', fontsize=11)
    axes[2, 1].set_ylabel('Airbrake Force (N)', fontsize=11)
    axes[2, 1].set_title('Airbrake Force vs Time', fontweight='bold')
    axes[2, 1].grid(True, alpha=0.3)
    
    plt.tight_layout()
    
    # Save figure
    plt.savefig('rocket_simulation_results.png', dpi=150, bbox_inches='tight')
    print("\nPlot saved to rocket_simulation_results.png")
    
    # Print summary statistics
    print("\n=== Simulation Summary ===")
    print(f"Max altitude: {np.max(positions):.2f} m")
    print(f"Target altitude: {TARGET_ALTITUDE:.2f} m")
    print(f"Final predicted apogee: {predicted_apogees[-1]:.2f} m")
    print(f"Max velocity: {np.max(velocities):.2f} m/s")
    print(f"Max acceleration: {np.max(accelerations)/G:.2f} G")
    print(f"Max airbrake force: {np.max(airbrake_forces):.2f} N")
    print(f"Flight time to apogee: {times[-1]:.3f} s")
    print(f"Final Cd estimate: {cd_estimates[-1]:.3f}")
    finite_target_cds = target_cds[np.isfinite(target_cds)]
    if finite_target_cds.size > 0:
        print(f"Final target Cd: {finite_target_cds[-1]:.3f}")
    print(f"Cd controller gains: Kp={CD_CTRL_KP:.2f}, Ki={CD_CTRL_KI:.2f}, deadband={CD_CTRL_ERR_DEADBAND:.3f}")
    
    return fig


if __name__ == '__main__':
    print("Starting rocket simulation...")
    print(f"Physics simulation: {DT*1000:.2f} ms ({1/DT:.0f} Hz)")
    print(f"Flight computer: {FC_DT*1000:.2f} ms ({1/FC_DT:.0f} Hz)")
    print(f"Estimator (Cd): {EST_DT*1000:.2f} ms ({1/EST_DT:.0f} Hz)")
    print(f"Motor max velocity: {MAX_MOTOR_VEL} units/s")
    print(f"Cd range: {CD_MIN} - {CD_MAX}")
    print()
    
    # Run simulation
    results = run_simulation()
    
    # Plot results
    fig = plot_results(*results)
    plt.show(block=False)
    plt.pause(0.1)
    print("\nSimulation complete! Close the plot window to exit.")
