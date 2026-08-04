import atexit, math, time #proqram bitende avtomatik ise dusmeni temin eden modul
import dronekit as dk #dronla esas unsiyyet kitabxanasi
from dronekit import LocationGlobalRelative
from pymavlink import mavutil #dronekit-in el catmadigi yerlerde istifade edilir

def wait_for(condition, timeout=10, interval=0.2, error=True):
    """
    This is a general helper function to wait for any condition to hold before continuing the program.
    It is a good practice, when programming drones, to wait until the command has been
    completed before proceeding to the next command. It uses time.monotonic() to note the
    current time. While some condition() is not true, it checks if timeout has been reached
    by subtracting new time (time.monotonic) - start. Then it waits "interval" seconds before
    checking the condition again (default is 0.2 seconds).
    Example usage: wait.for(lambda: vehicle.armed) -> it waits until the drone has been armed.
    Last parameter (error) determines if code should raise an error if it timeouts. True by default.
    """
    start = time.monotonic()
    while not condition():
        if timeout is not None and time.monotonic() - start >= timeout:
            if error:
                raise TimeoutError(f"Timed out after {timeout} seconds.")
            return False
        time.sleep(interval)

def connect_drone(connect_str):
    """
    This uses dronekit.connect interface to connect to drone. Here connection_str denotes the port/interface used
    to connect the drone. For example, on linux, to connect via USB, the connection string will be "/dev/ttyACM0".
    Here we also use python module "atexit". It registers a function (in our case vehicle.close) which will execute
    in the end of the code. It is to ensure that even if code crashes, vehicle.close() will run, which properly
    disconnects the drone.
    dronekit.connect() is the connection function. It returns a Vehicle object, which contains tons of methods to
    communicate with the drone. We assign it to "vehicle" variable which we will use to access its methods, like
    "vehicle.battery" or "vehicle.simple_takeoff()".
    """
    print(f"Connecting to {connect_str}.")
    vehicle = dk.connect(connect_str, wait_ready=True)
    atexit.register(vehicle.close)
    return vehicle

def get_telemetry(vehicle):
    """
    This is a helper function to get some useful internal data of the drone vehicle.

    Now also includes velocity and attitude, which weren't here before:
    - vehicle.velocity returns [vx, vy, vz] in m/s, in the NED frame
      (North, East, Down -- so vz is positive when descending).
    - vehicle.attitude returns roll/pitch/yaw in radians.
    Both of these will be needed later when we log data for the ML dataset
    (the drone's speed and tilt at release time affects where the payload lands).
    """
    bat = vehicle.battery
    status = vehicle.system_status
    vel = vehicle.velocity
    att = vehicle.attitude
    return {
            "mode": vehicle.mode.name if vehicle.mode else "UNKNOWN",
            "armed": vehicle.armed,
            "lat": vehicle.location.global_relative_frame.lat,
            "lon": vehicle.location.global_relative_frame.lon,
            "alt": vehicle.location.global_relative_frame.alt,
            "heading": vehicle.heading,
            "battery_level": bat.level if bat else -1,
            "gps": vehicle.gps_0.fix_type,
            "state": status.state if status else "UNKNOWN",
            "vx": vel[0] if vel else None,
            "vy": vel[1] if vel else None,
            "vz": vel[2] if vel else None,
            "roll": att.roll if att else None,
            "pitch": att.pitch if att else None,
            "yaw": att.yaw if att else None,
            }

def set_mode(vehicle, mode_name) -> None:
    """
    This is a function to change the mode of the drone. It does it by changing internal variable vehicle.mode to desired mode (mode_name).
    It also waits until the mode has been changed, because by default, the mode change takes some time.
    """
    print(f"Setting mode to {mode_name}...")
    vehicle.mode = dk.VehicleMode(mode_name)
    wait_for(lambda: vehicle.mode.name == mode_name)


#   The following functions are what makes the drone move.


def takeoff(vehicle, target_alt):
    """
    It also uses vehicle.location.global_relative_frame.alt to get the current altitude and waits
    until it is 95% there. We use 95% as a safety zone, in case drone sensors are not accurate.
    """
    print(f"Taking off to {target_alt}m...")
    vehicle.simple_takeoff(target_alt)
    wait_for(lambda: vehicle.location.global_relative_frame.alt >= target_alt * 0.95, timeout=20)
    print("Target altitude reached.")


def goto_position(vehicle, lat, lon, alt, groundspeed=None, timeout=60):
    """
    Flies the drone to a specific GPS coordinate (lat, lon) at a given
    relative altitude (alt, meters above the home/takeoff point).

    Vehicle must already be in GUIDED mode and airborne for this to work.

    Uses DroneKit's built-in vehicle.simple_goto(), which sends a position
    target to the flight controller and returns immediately -- it does NOT
    wait for arrival by itself. That's why we build our own small
    distance-check loop below and pass it to wait_for(), so the caller can
    choose to block until the drone actually arrives (or timeout expires).

    groundspeed (m/s) is optional. If not given, the drone flies at
    whatever speed is currently configured (WPNAV_SPEED parameter).
    """
    target = LocationGlobalRelative(lat, lon, alt)
    print(f"Going to {lat}, {lon} at {alt}m...")
    if groundspeed:
        vehicle.simple_goto(target, groundspeed=groundspeed)
    else:
        vehicle.simple_goto(target)

    def distance_to_target():
        """
        Simple flat-earth approximation: converts the lat/lon difference
        (in degrees) to meters. Accurate enough for short distances (up to
        a few hundred meters) -- not meant for long-range navigation.
        111320 is roughly how many meters are in one degree of latitude.
        For longitude, we additionally multiply by cos(latitude), because
        longitude degrees get "narrower" the further you are from the equator.
        """
        current = vehicle.location.global_relative_frame
        dlat = (target.lat - current.lat) * 111320
        dlon = (target.lon - current.lon) * 111320 * math.cos(math.radians(current.lat))
        return math.sqrt(dlat**2 + dlon**2)

    reached = wait_for(lambda: distance_to_target() < 1.0, timeout=timeout, error=False)
    if reached:
        print("Reached target position.")
    else:
        print("Warning: goto_position timed out before reaching target.")


def release_payload(vehicle, servo_channel, release_pwm=1900, hold_pwm=1100, hold_time=1.0):
    """
    Releases the payload by driving a servo to the "open" PWM position,
    holding it there briefly, then returning it to the "closed" position.

    Like spin_yaw(), DroneKit has no built-in "release payload" method, so
    we send a raw MAVLink command via vehicle.message_factory. The command
    used here is MAV_CMD_DO_SET_SERVO, documented at:
    https://mavlink.io/en/messages/common.html#MAV_CMD_DO_SET_SERVO

    It directly commands one servo output channel to a given PWM value
    (in microseconds), same as writing a PWM value in Mission Planner's
    "Motor Test" / servo output screen.

    Parameters:
    - servo_channel: which output channel the release servo is wired to
      on the flight controller (check your wiring and the SERVOx_FUNCTION
      parameter in Mission Planner -- it must be set to "RCPassThru" or
      similar so the autopilot doesn't override it).
    - release_pwm: PWM value that opens/triggers the release mechanism.
    - hold_pwm: PWM value the servo returns to afterward (closed/resting).
    - hold_time: how long (seconds) to keep the servo at release_pwm
      before returning it to hold_pwm.

    NOTE: release_pwm and hold_pwm depend entirely on your physical servo
    and release mechanism -- test carefully on the ground first, without
    a payload attached, to find the correct values.
    """
    print(f"Releasing payload on channel {servo_channel}...")

    def set_servo(pwm):
        msg = vehicle.message_factory.command_long_encode(
                0, 0,
                mavutil.mavlink.MAV_CMD_DO_SET_SERVO,
                0,
                servo_channel,  # which servo output channel
                pwm,            # PWM value to set, in microseconds
                0, 0, 0, 0, 0)  # unused parameters
        vehicle.send_mavlink(msg)

    set_servo(release_pwm)
    time.sleep(hold_time)
    set_servo(hold_pwm)
    print("Payload released.")


def spin_yaw(vehicle, angle, speed, report=False, timeout=15):
    """
    This is currently the most complicated function we have. Dronekit library does not support simple yaw (spinning) action
    for the drone. In order to achieve it, we use MAVLINK protocol directly.

    Learn about MAVLINK protocol here: https://mavlink.io/en/

    Corresponding mavlink command to make the drone yaw is MAV_CMD_CONDITION_YAW. Parameters clearly docummented here:
    https://mavlink.io/en/messages/common.html#MAV_CMD_CONDITION_YAW

    To send a raw mavlink message to the drone, we use vehicle.message_factory.command_long_encode() method.
    Parameters are explained in the comments below.

    timeout added: if the drone can't complete the full turn within
    `timeout` seconds (e.g. it gets stuck or the heading reading is noisy),
    we give up waiting instead of looping forever.
    """
    print(f"Spinning {angle} degrees at {speed} deg/s...")
    msg = vehicle.message_factory.command_long_encode(
            0, 0, # These are target system and component. First 0 is ignored, second 0 indicates autopilot.
            mavutil.mavlink.MAV_CMD_CONDITION_YAW, # Command
            0, # 0 means no repeated confirmation is needed to execute this command
            angle, # turning angle, in degrees
            speed, # angular velocity, in degrees / second
            1, # turning direction, 1 - clockwise, -1 - counter-clockwise
            1, # 0 - absolute (exact compass angle), 1 - relative (turning degrees relative to previous angle)
            0, 0, 0) # Last three zeros are unused parameters
    vehicle.send_mavlink(msg) # We built and stored the message in msg variable, which this line sends to the drone

#   The following lines are to check if the drone actually turned needed amount of degrees.

    prev = vehicle.heading # vehicle.heading is the current compass angle
    total = 0 # total it turned, begins with 0
    start = time.monotonic()

    while total < (angle - 5):
        if time.monotonic() - start >= timeout:
            print("Warning: spin_yaw timed out, continuing anyway.")
            break
        curr = vehicle.heading # read the current heading angle
        if report: # Report parameter determines whether to print the current angle each time it checks. Disabled (false) by default.
            print("Heading", curr)

        """
        The following formula calculates the difference between previous angle and current angle.
        It takes into account that we only have 360 degrees. If drone turns from 355 clockwise 10 degrees, new angle is 5 degrees,
        but naive curr - prev will return 5 - 355 = -350, which is obviously wrong. The formula adjusts it.
        """
        diff = (curr - prev + 180) % 360 - 180
        total += diff
        prev = curr
        time.sleep(0.1)

    print("Spin complete.")


def land(vehicle):
    """
    This function safely lands the drone and waits until it is landed. It does it by changing the drone mode to "LAND".
    "LAND" automatically disarms the drone. We use this fact to wait until the drone is disarmed before continuing.
    """
    print("Landing...")
    vehicle.mode = dk.VehicleMode("LAND")
    wait_for(lambda: not vehicle.armed, timeout=60)
    print("Landed and disarmed.")