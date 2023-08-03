import cv2
import numpy as np


def get_frames(file):
    video = cv2.VideoCapture(file)

    output = []

    while True:
        ret, frame = video.read()

        if not ret:
            break

        output.append(frame)

    return output


def detect_april_tags(img, at_detector):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    cameraMatrix = np.array(
        [353.571428571, 0, 320, 0, 353.571428571, 180, 0, 0, 1]
    ).reshape((3, 3))
    camera_params = (
        cameraMatrix[0, 0],
        cameraMatrix[1, 1],
        cameraMatrix[0, 2],
        cameraMatrix[1, 2],
    )

    tags = at_detector.detect(
        gray, estimate_tag_pose=True, camera_params=camera_params, tag_size=0.1
    )

    return tags


def determine_error(tags, shape):
    if (type(tags) != list) or (type(shape) != tuple or len(shape) != 2):
        raise ValueError(
            "Arguments must be a list of tags, and a tuple of size two corresponding to the width and height of the image respectively."
        )

    width, height = shape

    try:
        x_errors = [(tag.center[0] - (width / 2)) for tag in tags]
        y_errors = [(tag.center[1] - (height / 2)) for tag in tags]

        x_avg_error = sum(x_errors) / len(x_errors)
        y_avg_error = sum(y_errors) / len(y_errors)

        x_avg_error /= width
        y_avg_error /= height

    except Exception as e:
        raise e

    # return x_avg_error, y_avg_error

    return x_errors, y_errors, x_avg_error, y_avg_error


def calculate_pid_output(errors, pid_controllers):
    x_error, y_error = errors
    x_pid_controller, y_pid_controller = pid_controllers

    output = (x_pid_controller.update(x_error), y_pid_controller.update(y_error))

    return output


def processed_image(img, tags, x_errors, y_errors):
    x_center, y_center = map(int, ((img.shape[1] / 2), (img.shape[0] / 2)))
    crosshair_length = int(img.shape[1] / 37)

    processed_img = img.copy()

    for tag in tags:
        for idx in range(len(tag.corners)):
            cv2.line(
                processed_img,
                tuple(tag.corners[idx - 1, :].astype(int)),
                tuple(tag.corners[idx, :].astype(int)),
                (0, 255, 0),
            )

        cv2.line(
            processed_img,
            (x_center, y_center - crosshair_length),
            (x_center, y_center + crosshair_length),
            (255, 0, 0),
            5,
        )

        cv2.line(
            processed_img,
            (x_center - crosshair_length, y_center),
            (x_center + crosshair_length, y_center),
            (255, 0, 0),
            5,
        )

        cv2.line(
            processed_img,
            (x_center, y_center),
            (
                x_center + int(x_errors[tags.index(tag)]),
                y_center + int(y_errors[tags.index(tag)]),
            ),
            (0, 255, 0),
            5,
        )

        cv2.putText(
            processed_img,
            str(tag.tag_id),
            org=(
                tag.corners[0, 0].astype(int) + 10,
                tag.corners[0, 1].astype(int) + 10,
            ),
            fontFace=cv2.FONT_HERSHEY_SIMPLEX,
            fontScale=0.8,
            color=(0, 0, 255),
        )

    return processed_img
