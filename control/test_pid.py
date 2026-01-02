import time
from pid import PID, PIDConfig

# Example config — conservative
cfg = PIDConfig(
    kp=1.2,
    ki=0.0,
    kd=0.15,
    u_min=-0.2,
    u_max=0.2,
    d_cutoff_hz=5.0,
)

pid = PID(cfg)

d_ref = 0.40     # target distance [m]
d_meas = 0.60    # pretend robot starts far away

dt = 0.05        # 20 Hz loop

for k in range(60): #60 discrete "time steps" or iterations of the control loop.
    print(pid.debug_str(d_ref, d_meas, dt)) #this line must be removed in real pi-pico pid controller testing

    # Fake plant response (distance decreases when u > 0)
    # subtracting the product of the controller's most recent output and the time step.
    d_meas -= pid.last_output * dt

    time.sleep(dt)
