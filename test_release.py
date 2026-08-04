import threading
import time
import drone_utils as drone

CONNECTION_STRING = "udp:0.0.0.0:14550"
SERVO_CHANNEL = 9  # must match whichever channel you plan to wire the release servo to


def main():
    vehicle = drone.connect_drone(CONNECTION_STRING)

    # We use a dict (not a plain variable) so the listener function below
    # can update it -- Python closures can read outer variables but can't
    # reassign a plain variable from inside a nested function.
    current_pwm = {"value": None}

    def servo_listener(self, name, message):
        """
        DroneKit calls this automatically every time SITL sends a
        SERVO_OUTPUT_RAW message (this happens continuously, many times per
        second, regardless of what our code is doing -- it's the autopilot
        reporting its actual current outputs). We just grab the field for
        the channel we care about and store it.
        """
        field_name = f"servo{SERVO_CHANNEL}_raw"
        current_pwm["value"] = getattr(message, field_name, None)

    vehicle.add_message_listener('SERVO_OUTPUT_RAW', servo_listener)

    print("Waiting for the first SERVO_OUTPUT_RAW message from SITL...")
    drone.wait_for(lambda: current_pwm["value"] is not None, timeout=10)
    print(f"Baseline (before release): channel {SERVO_CHANNEL} = {current_pwm['value']}")

    # release_payload() is blocking (it sleeps internally for hold_time),
    # so to actually SEE the value change while it's running, we print it
    # from a separate background thread instead of the main thread.
    stop_monitor = threading.Event()

    def monitor():
        while not stop_monitor.is_set():
            print(f"  live reading -- channel {SERVO_CHANNEL} = {current_pwm['value']}")
            time.sleep(0.3)

    monitor_thread = threading.Thread(target=monitor, daemon=True)
    monitor_thread.start()

    # hold_time=2.0 (longer than the default) just so the monitor thread
    # above has time to print a few readings while it's in the "released" state.
    drone.release_payload(vehicle, servo_channel=SERVO_CHANNEL,
                           release_pwm=1900, hold_pwm=1100, hold_time=2.0)

    # Give SITL a moment to actually send us a fresh SERVO_OUTPUT_RAW
    # message reflecting the hold_pwm command -- release_payload() returns
    # the instant it SENDS that command, not once we've received
    # confirmation of it back over telemetry. Without this wait, we'd be
    # reading a value that's still "in transit".
    time.sleep(1.0)

    stop_monitor.set()
    monitor_thread.join()

    print(f"After release: channel {SERVO_CHANNEL} = {current_pwm['value']}")
    print("\nExpected pattern: baseline -> jumps to 1900 during release -> back to 1100 after.")
    print("If the number never changes, the command isn't reaching that channel -- see notes below.")


if __name__ == "__main__":
    main()