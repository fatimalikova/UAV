import math
import random
import csv

G = 9.81  # gravitational acceleration, m/s^2


def predict_drift(altitude, drone_speed, drone_heading_deg, wind_speed, wind_heading_deg):
    """
    Simplified physics model: given the conditions AT THE MOMENT OF RELEASE,
    predicts how far (and in which direction) the payload will drift
    horizontally from the point directly below the drone, by the time it
    hits the ground.

    This is a SIMPLIFIED model on purpose (matches what the roadmap calls
    "Xett A"): it assumes free fall (no air resistance on the vertical
    fall), and that both the drone's own horizontal velocity and the wind
    push the payload sideways at a CONSTANT rate for the whole fall time.
    Real payloads experience drag, which slows this sideways push down
    over time -- that gap between this simple model and reality is exactly
    what the real-world test data (30/40/50m drops) will later correct,
    when we calibrate the ML model on real measurements.

    Parameters:
    - altitude: release height, in meters (above the ground/target)
    - drone_speed: drone's ground speed at release, in m/s
    - drone_heading_deg: direction the drone is flying, in degrees
      (0 = North, 90 = East, standard compass bearing)
    - wind_speed: wind speed, in m/s
    - wind_heading_deg: direction the wind is blowing TOWARD, in degrees
      (same convention as drone_heading_deg)

    Returns:
    - (drift_east, drift_north): how far the payload lands from directly
      below the release point, in meters, in a flat East/North frame.
      Positive drift_east = lands to the East of the release point.
      Positive drift_north = lands to the North of the release point.
    """
    # Step 1: how long the payload falls, ignoring air resistance.
    # From basic kinematics: altitude = 0.5 * g * t^2  =>  t = sqrt(2*altitude/g)
    t_fall = math.sqrt(2 * altitude / G)

    # Step 2: convert compass headings (degrees, 0=North, clockwise) into
    # standard East/North vector components using sin/cos.
    drone_heading_rad = math.radians(drone_heading_deg)
    wind_heading_rad = math.radians(wind_heading_deg)

    # Step 3: how far the drone's own forward motion carries the payload
    # sideways during the fall (payload keeps the drone's velocity at the
    # moment of release, same as a bomb dropped from a moving plane).
    drone_drift_east = drone_speed * math.sin(drone_heading_rad) * t_fall
    drone_drift_north = drone_speed * math.cos(drone_heading_rad) * t_fall

    # Step 4: how far the wind pushes the payload sideways during the fall.
    wind_drift_east = wind_speed * math.sin(wind_heading_rad) * t_fall
    wind_drift_north = wind_speed * math.cos(wind_heading_rad) * t_fall

    # Step 5: total drift = both effects combined (vector addition).
    drift_east = drone_drift_east + wind_drift_east
    drift_north = drone_drift_north + wind_drift_north

    return drift_east, drift_north


def release_point_offset(target_east, target_north, altitude, drone_speed,
                          drone_heading_deg, wind_speed, wind_heading_deg):
    """
    The INVERSE of predict_drift(): given a target location and the current
    conditions, calculates WHERE the drone should release the payload
    (relative to the target) so that it lands ON the target.

    This is what mission.py will actually use in flight: it flies to
    (target - drift) instead of straight to the target, then releases.
    """
    drift_east, drift_north = predict_drift(
        altitude, drone_speed, drone_heading_deg, wind_speed, wind_heading_deg)
    release_east = target_east - drift_east
    release_north = target_north - drift_north
    return release_east, release_north


def generate_dataset(filename="ballistic_dataset.csv", n_samples=5000, seed=42):
    """
    Runs predict_drift() thousands of times with randomized, realistic
    input ranges, and saves every (inputs -> drift) pair to a CSV file.
    This CSV becomes the training data for the ML model (Xett B).

    The ranges below are rough starting points -- adjust them to match
    your actual F450's real flight envelope (max speed, typical release
    altitude, realistic wind conditions for your test site).
    """
    random.seed(seed)  # fixed seed = reproducible dataset (same data every run)

    rows = []
    for _ in range(n_samples):
        altitude = random.uniform(10, 50)          # release height, meters
        drone_speed = random.uniform(0, 15)         # drone ground speed, m/s
        drone_heading = random.uniform(0, 360)       # degrees
        wind_speed = random.uniform(0, 10)           # wind speed, m/s
        wind_heading = random.uniform(0, 360)        # degrees

        drift_east, drift_north = predict_drift(
            altitude, drone_speed, drone_heading, wind_speed, wind_heading)

        rows.append({
            "altitude": altitude,
            "drone_speed": drone_speed,
            "drone_heading": drone_heading,
            "wind_speed": wind_speed,
            "wind_heading": wind_heading,
            "drift_east": drift_east,
            "drift_north": drift_north,
        })

    with open(filename, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)

    print(f"Generated {n_samples} samples, saved to {filename}")


if __name__ == "__main__":
    # Quick sanity check with one example before generating the full dataset
    drift = predict_drift(altitude=20, drone_speed=5, drone_heading_deg=90,
                           wind_speed=3, wind_heading_deg=0)
    print(f"Example: at 20m altitude, 5m/s East, 3m/s wind from South ->")
    print(f"  drift = {drift[0]:.2f}m East, {drift[1]:.2f}m North")

    generate_dataset()