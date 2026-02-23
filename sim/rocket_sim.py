import numpy as np
import matplotlib.pyplot as plt
from coast_table import get_remaining_altitude, CD_RANGE, ALTITUDE_TABLE

# Constants from config.h
MASS = 0.603  # kg
BASE_CD = 0.44  # From OpenRocket
AREA = (np.pi * (6.6 / 100.0) ** 2) / 4.0  # m^2
RHO = 1.225  # kg/m^3
G = 9.80665  # m/s^2
rhoA = RHO * AREA

# Simulation parameters
DT = 0.001  # 1 kHz physics simulation
FLIGHT_COMP_DT = 0.002  # 500 Hz flight computer & Cd estimator
MAX_MOTOR_VEL = 60.0  # units/s (0-24 in 0.4s)
MOTOR_MIN = 0.0
MOTOR_MAX = 24.0

# Cd relationship: linear from BASE_CD at position 0 to 4.0 at position 24
CD_MIN = BASE_CD
CD_MAX = 4.0

# Cd estimator parameters (from estimator.cpp)
ALPHA_CD = 0.05  # Low-pass filter coefficient at 500 Hz
CONTROL_VEL_START = 55.0  # m/s, velocity threshold for deploying airbrakes
TARGET_ALTITUDE = 228.6  # m, target apogee altitude


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


def flight_computer(pos, vel, accel, cd_estimate, motor_position):
    """
    Flight computer algorithm with optimal control.
    Uses coast phase lookup table to find optimal Cd, then calculates
    required motor position based on current drag characteristics.
    
    Returns target motor position.
    """
    # Only activate control during coast phase
    if accel >= 0 or vel > CONTROL_VEL_START:
        return MOTOR_MIN
    
    # Predict apogee with current conditions
    remaining_alt = get_remaining_altitude(vel, cd_estimate)
    predicted_apogee = pos + remaining_alt
    
    # Find optimal Cd for target altitude
    # Binary search through Cd range to find Cd that gives target altitude
    optimal_cd = cd_estimate
    
    # Simple search: try different Cd values
    best_cd = cd_estimate
    min_error = abs(predicted_apogee - TARGET_ALTITUDE)
    
    for cd_test in np.linspace(CD_MIN, CD_MAX, 20):
        test_remaining = get_remaining_altitude(vel, cd_test)
        test_apogee = pos + test_remaining
        error = abs(test_apogee - TARGET_ALTITUDE)
        
        if error < min_error:
            min_error = error
            best_cd = cd_test
    
    optimal_cd = best_cd
    
    # Calculate drag per unit position from current state
    # cd_estimate = BASE_CD + drag_per_position * motor_position
    # So: drag_per_position = (cd_estimate - BASE_CD) / motor_position
    
    if motor_position > 1.0:  # Only calculate if motor is significantly open
        drag_per_position = (cd_estimate - BASE_CD) / motor_position
    else:
        # Use default relationship if motor is near zero
        drag_per_position = (CD_MAX - CD_MIN) / MOTOR_MAX
    
    # Calculate required motor position for optimal Cd
    # optimal_cd = BASE_CD + drag_per_position * required_position
    if abs(drag_per_position) > 1e-6:  # Avoid division by zero
        required_position = (optimal_cd - BASE_CD) / drag_per_position
    else:
        required_position = MOTOR_MIN
    
    # Clamp to valid range
    required_position = np.clip(required_position, MOTOR_MIN, MOTOR_MAX)
    
    return required_position


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
    
    # Storage for plotting
    times = []
    positions = []
    velocities = []
    accelerations = []
    motor_positions = []
    target_motor_positions = []
    cd_estimates = []
    airbrake_forces = []
    predicted_apogees = []
    
    # Flight computer timing (500 Hz)
    last_fc_time = 0.0
    
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
        
        # Run flight computer and Cd estimator at 500 Hz
        if t - last_fc_time >= FLIGHT_COMP_DT - 1e-9:  # Small epsilon for float comparison
            # Update Cd estimate
            cd_estimate = update_cd_estimate(cd_estimate, accel, state[1])
            
            # Flight computer
            target_motor_position = flight_computer(state[0], state[1], accel, cd_estimate, motor_position)
            
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
        
        # Update motor position with rate limiting (runs at 1 kHz)
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
            np.array(airbrake_forces), np.array(predicted_apogees))


def plot_results(times, positions, velocities, accelerations, 
                motor_positions, target_motor_positions, cd_estimates,
                airbrake_forces, predicted_apogees):
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
    
    return fig


if __name__ == '__main__':
    print("Starting rocket simulation...")
    print(f"Physics simulation: {DT*1000:.1f} ms ({1/DT:.0f} Hz)")
    print(f"Flight computer: {FLIGHT_COMP_DT*1000:.1f} ms ({1/FLIGHT_COMP_DT:.0f} Hz)")
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
