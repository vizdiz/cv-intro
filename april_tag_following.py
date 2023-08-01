import cv2
from pymavlink import mavutil
from dt_apriltags import Detector
import matplotlib.pyplot as plt
import numpy as np
from pid import PID
import sys
import signal
from video import Video


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

    set_rc_channel_pwm(mav, 4, 1500 + power * 5)


def set_y_axis_power(mav, power=0):
    """Set vertical power
    Args:
        power (int, optional): Vertical power value -100-100
    """
    if power < -100 or power > 100:
        print("Power value out of range. Clipping...")
        power = np.clip(power, -100, 100)

    power = int(power)

    set_rc_channel_pwm(mav, 5, 1500 + power * 5)


def press_to_depth(pressure):
    """Convert pressure to depth
    Args:
        pressure (float): Pressure in hPa
    Returns:
        float: Depth in water in meters
    """
    rho = 1029  # density of fresh water in kg/m^3
    g = 9.81  # gravity in m/s^2
    pressure_at_sea_level = 1013.25  # pressure at sea level in hPa
    # multiply by 100 to convert hPa to Pa
    return (pressure - pressure_at_sea_level) * 100 / (rho * g)


def main():
    mav = mavutil.mavlink_connection("udpin:0.0.0.0:14550")

    x_pid = PID(0.45, 0.0, 0.1)
    y_pid = PID(0.30, 0.0, 0.075)

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
        processed_image, tags, raw_x_error, raw_y_error = detect_april_tags(frame)

        if len(tags) < 1:
            continue

        # calculate error
        x_output = x_pid.update(raw_x_error)
        y_output = y_pid.update(raw_y_error)

        print("X-Axis Output: ", x_output)
        print("Y-Axis Output: ", y_output)

        # set x-axis power
        set_x_axis_power(mav, x_output)

        # set y-axis power

        set_y_axis_power(mav, y_output)


def detect_april_tags(img, at_detector):
    try:
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        cameraMatrix = np.array([1060.71, 0, 960, 0, 1060.71, 540, 0, 0, 1]).reshape(
            (3, 3)
        )
        camera_params = (
            cameraMatrix[0, 0],
            cameraMatrix[1, 1],
            cameraMatrix[0, 2],
            cameraMatrix[1, 2],
        )

        tags = at_detector.detect(
            gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1
        )

        color_img = cv2.cvtColor(gray, cv2.COLOR_GRAY2RGB)

        x_errors = [(tag.center[0] - (img.shape[1] / 2)) for tag in tags]
        y_errors = [(tag.center[1] - (img.shape[0] / 2)) for tag in tags]

        # print((x_errors, y_errors))

        x_center, y_center = int(img.shape[1] / 2), int(img.shape[0] / 2)
        crosshair_length = int(img.shape[1] / 20)

        for tag in tags:
            for idx in range(len(tag.corners)):
                cv2.line(
                    color_img,
                    tuple(tag.corners[idx - 1, :].astype(int)),
                    tuple(tag.corners[idx, :].astype(int)),
                    (0, 255, 0),
                )

            cv2.line(
                color_img,
                (x_center, y_center - crosshair_length),
                (x_center, y_center + crosshair_length),
                (255, 0, 0),
                5,
            )

            cv2.line(
                color_img,
                (x_center - crosshair_length, y_center),
                (x_center + crosshair_length, y_center),
                (255, 0, 0),
                5,
            )

            cv2.line(
                color_img,
                (x_center, y_center),
                (
                    x_center + int(x_errors[tags.index(tag)]),
                    y_center + int(y_errors[tags.index(tag)]),
                ),
                (0, 255, 0),
                5,
            )

            cv2.putText(
                color_img,
                str(tag.tag_id),
                org=(
                    tag.corners[0, 0].astype(int) + 10,
                    tag.corners[0, 1].astype(int) + 10,
                ),
                fontFace=cv2.FONT_HERSHEY_SIMPLEX,
                fontScale=0.8,
                color=(0, 0, 255),
            )

        x_avg_error = sum(x_errors) / len(x_errors)
        y_avg_error = sum(y_errors) / len(y_errors)

        return color_img, tags, x_avg_error, y_avg_error

    except Exception as e:
        print(e)

        return img, None, None, None

    # print(f"x error: {x_avg_error}")
    # print(f"y error: {y_avg_error}")

    # plt.imshow(color_img)
    # plt.show()


if __name__ == "__main__":
    main()
