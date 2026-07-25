import drone_utils as drone # we import our helper functions from drone_utils.py file

# CONNECTION_STRING = "udp:127.0.0.1:14550" # In simulator, we use UDP protocol to connect the drone.
CONNECTION_STRING = "udp:0.0.0.0:14550"

# The following is generally a "good python practice" we follow for executable scripts
# as opposed to helper files like our drone_utils.py.
# We wrap the entire code into main() function, and below check if the file is main, we run it.
# It does not really affect anything much, it is just recommended python style.

def main():
    vehicle = drone.connect_drone(CONNECTION_STRING) # connect to the drone

    print("Initial telemetry:", drone.get_telemetry(vehicle)) # get initial telemetry data
    
    drone.set_mode(vehicle, "GUIDED") # Generally, we change the drone mode to GUIDED in order to control it safely and make it armable.
    vehicle.armed = True # Arm the drone (to activate motors)
    drone.wait_for(lambda: vehicle.armed) # Wait until it is armed
    print("Drone is armed!")
    drone.takeoff(vehicle, 3) # Take off the drone 3 meters up
    drone.spin_yaw(vehicle, 360, 60) # Spin drone 360 degrees, 60 deg/s
    drone.land(vehicle) # Land it

    print("Mission completed successfully.")

# This is the standard pattern for that python practice
if __name__ == "__main__":
    main()

