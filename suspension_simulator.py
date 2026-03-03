import numpy as np
import matplotlib.pyplot as plt


# =====================================
# SYSTEM PARAMETERS
# =====================================

m = 250        # kg (quarter car mass)
k = 15000      # N/m (spring stiffness)
c = 4000       # Ns/m (damping coefficient)

# =====================================
# ENGINEERING ANALYSIS
# =====================================

omega_n = np.sqrt(k / m)
zeta = c / (2 * np.sqrt(k * m))

print("===== SYSTEM ANALYSIS =====")
print(f"Natural Frequency (rad/s): {omega_n:.2f}")
print(f"Damping Ratio: {zeta:.3f}")

if zeta < 1:
    print("System Type: Underdamped")
elif zeta == 1:
    print("System Type: Critically Damped")
else:
    print("System Type: Overdamped")

print("===========================\n")

# =====================================
# SIMULATION SETTINGS
# =====================================

dt = 0.001
t_end = 5
t = np.arange(0, t_end, dt)

# Initial Conditions
x = 0      # displacement (m)
v = 0      # velocity (m/s)

x_list = []
r_list = []

# =====================================
# ACCELERATION FUNCTION
# =====================================

def acceleration(x, v, r):
    r_dot = 0  # assuming flat bump edges
    return (-c * (v - r_dot) - k * (x - r)) / m

# =====================================
# SIMULATION LOOP (RK4 METHOD)
# =====================================

for time in t:

    # Road bump input (5 cm bump)
    if 0.5 < time < 0.6:
        r = 0.05
    else:
        r = 0

    # RK4 Integration

    k1_v = acceleration(x, v, r)
    k1_x = v

    k2_v = acceleration(x + 0.5 * dt * k1_x,
                        v + 0.5 * dt * k1_v, r)
    k2_x = v + 0.5 * dt * k1_v

    k3_v = acceleration(x + 0.5 * dt * k2_x,
                        v + 0.5 * dt * k2_v, r)
    k3_x = v + 0.5 * dt * k2_v

    k4_v = acceleration(x + dt * k3_x,
                        v + dt * k3_v, r)
    k4_x = v + dt * k3_v

    x += (dt / 6) * (k1_x + 2*k2_x + 2*k3_x + k4_x)
    v += (dt / 6) * (k1_v + 2*k2_v + 2*k3_v + k4_v)

    x_list.append(x)
    r_list.append(r)

# =====================================
# PLOT RESULTS
# =====================================

plt.figure(figsize=(10, 6))
plt.plot(t, x_list, label="Car Body Displacement")
plt.plot(t, r_list, label="Road Input", linestyle='--')
plt.xlabel("Time (s)")
plt.ylabel("Displacement (m)")
plt.title("Vehicle Suspension Response (RK4)")
plt.legend()
plt.grid()
plt.show()

from matplotlib.animation import FuncAnimation

# =====================================
# ANIMATION
# =====================================

fig, ax = plt.subplots()
ax.set_xlim(-0.5, 0.5)
ax.set_ylim(-0.1, 0.2)

car_body, = ax.plot([], [], 's', markersize=20, label="Car Body")
road_line, = ax.plot([], [], linewidth=4, label="Road")

spring_line, = ax.plot([], [], linewidth=2)

ax.legend()
ax.set_title("Suspension Animation")

def update(frame):
    x = x_list[frame]
    r = r_list[frame]

    # Car position
    car_body.set_data(0, x)

    # Road
    road_line.set_data([-0.5, 0.5], [r, r])

    # Spring (just a simple line)
    spring_line.set_data([0, 0], [r, x])

    return car_body, road_line, spring_line

ani = FuncAnimation(fig, update, frames=len(t), interval=1)

plt.show()

# =====================================
# FREQUENCY RESPONSE ANALYSIS
# =====================================

frequencies = np.linspace(0.1, 20, 100)
amplitudes = []

A = 0.01  # 1 cm sinusoidal road input

for omega in frequencies:

    x = 0
    v = 0

    steady_state_x = []

    for time in t:

        r = A * np.sin(omega * time)
        r_dot = A * omega * np.cos(omega * time)

        def accel(x, v):
            return (-c * (v - r_dot) - k * (x - r)) / m

        # RK4
        k1_v = accel(x, v)
        k1_x = v

        k2_v = accel(x + 0.5*dt*k1_x, v + 0.5*dt*k1_v)
        k2_x = v + 0.5*dt*k1_v

        k3_v = accel(x + 0.5*dt*k2_x, v + 0.5*dt*k2_v)
        k3_x = v + 0.5*dt*k2_v

        k4_v = accel(x + dt*k3_x, v + dt*k3_v)
        k4_x = v + dt*k3_v

        x += (dt/6)*(k1_x + 2*k2_x + 2*k3_x + k4_x)
        v += (dt/6)*(k1_v + 2*k2_v + 2*k3_v + k4_v)

        steady_state_x.append(x)

    amplitudes.append(max(np.abs(steady_state_x[-1000:])))

plt.figure()
plt.plot(frequencies, amplitudes)
plt.xlabel("Excitation Frequency (rad/s)")
plt.ylabel("Response Amplitude (m)")
plt.title("Frequency Response of Suspension")
plt.grid()
plt.show()