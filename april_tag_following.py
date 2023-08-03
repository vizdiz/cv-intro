from pymavlink import mavutil
from dt_apriltags import Detector
import numpy as np
from pid import PID
import sys
import signal
from video import Video
from april_tag_processing import *


def set_rc_channel_pwm(mav, channel_id, pwm=1500):
    """Set RC channel pwm value
    Args:
        channel_id (TYPE): Channel ID
        pwm (int, optional): Channel pwm value 1100-1900
    """
    if channel_id < 1 or channel_id > 18:
        print("Channel does not exist.")
        return

    # Mavlink 2 supports up to 18 channels:
    # https://mavlink.io/en/messages/common.html#RC_CHANNELS_OVERRIDE
    rc_channel_values = [65535 for _ in range(18)]
    rc_channel_values[channel_id - 1] = pwm
    mav.mav.rc_channels_override_send(
        mav.target_system,  # target_system
        mav.target_component,  # target_component
        *rc_channel_values,
    )


def set_x_axis_power(mav, power=0):
    """Set vertical power
    Args:
        power (int, optional): Vertical power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range. Clipping...")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 5, 1500 + power * 5)


def set_y_axis_power(mav, power=0):
    """Set vertical power
    Args:
        power (int, optional): Vertical power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range. Clipping...")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 3, 1500 + power * 5)


def main():
    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    x_pid = PID(10.0, 0.0, 1.0)
    y_pid = PID(9.0, 0.0, 2.0)

    # catch CTRL+C
    def signal_handler(sig, frame):
        print("CTRL+C pressed. Disarming")
        mav.arducopter_disarm()
        mav.motors_disarmed_wait()
        print("Disarmed")
        sys.exit(0)

    # catch CTRL+C
    signal.signal(signal.SIGINT, signal_handler)

    # wait for the heartbeat message to find the system ID
    mav.wait_heartbeat()

    # arm the vehicle
    print("Arming")
    mav.arducopter_arm()
    mav.motors_armed_wait()
    print("Armed")

    # set mode to MANUAL
    print("Setting mode to MANUAL")
    mav.mav.set_mode_send(
        mav.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        19,  # Manual mode
    )
    print("Mode set to MANUAL")

    at_detector = at_detector = Detector(
        families="tag36h11",
        nthreads=1,
        quad_decimate=1.0,
        quad_sigma=0.0,
        refine_edges=1,
        decode_sharpening=0.25,
        debug=0,
    )

    while True:
        video = Video()

        print("Initialising stream...")

        waited = 0

        while not video.frame_available():
            waited += 1

            # print("\r  Frame not available (x{})".format(waited), end="")

            # sleep(0.1)
            # cv2.waitKey(30)

            if waited > 100:
                print("Stream retrieval failed.")
                break

        if video.frame_available():
            print('\nSuccess!\nStarting streaming - press "q" to quit.')
            # Only retrieve and display a frame if it's new
            frame = video.frame()
        else:
            continue

        # calculate error in pixels on both the X and Y axes
        height, width, channels = frame.shape

        tags = detect_april_tags(frame, at_detector)

        x_raw_error, y_raw_error = determine_error(tags, (width, height))

        errors = (x_raw_error, y_raw_error)
        pid_controllers = (x_pid, y_pid)

        x_output, y_output = calculate_pid_output(errors, pid_controllers)

        print(f"X-output{x_output}, Y-output:{y_output}")


# TODO: Add thread implementation to run image retrieval and motors concurrently


if __name__ == "__main__":
    main()
