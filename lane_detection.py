import cv2
import numpy as np


def detect_lines(
    img,
    threshold1: int = 50,
    threshold2: int = 100,
    apertureSize: int = 3,
    minLineLength: int = 100,
    maxLineGap: int = 10,
):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # convert to grayscale
    edges = cv2.Canny(
        gray, threshold1, threshold2, apertureSize=apertureSize
    )  # detect edges
    lines = cv2.HoughLinesP(
        edges,
        1,
        np.pi / 180,
        100,
        minLineLength=minLineLength,
        maxLineGap=maxLineGap,
    )  # detects lines

    return lines


def draw_lines(img, lines, color: tuple):
    for line in lines:
        x1, y1, x2, y2 = line[0]
        cv2.line(img, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)

    return img


def get_slopes_intercepts(lines, height):
    slopes = []
    intercepts = []

    m_tol = 1e2
    dx_tol = 1e-3

    for line in lines:
        x1, y1, x2, y2 = line[0]

        if abs(x2 - x1) < dx_tol:
            dx = max(dx_tol, (x2 - x1)) if (x2 - x1 > 0) else min(-dx_tol, (x2 - x1))
        else:
            dx = x2 - x1

        m = np.clip((y2 - y1) / dx, -m_tol, m_tol)
        b = x1 + ((height - y1) / m)

        slopes.append(m)
        intercepts.append(b)

    return slopes, intercepts


def detect_lanes(lines, height):
    slopes, intercepts = get_slopes_intercepts(lines, height)

    m_tol = 15e-1
    b_tol = 6e2

    dm_min = 1e-2
    db_min = 5e1

    horizontal_tol = 1e3
    vertical_threshold = 5e2

    used = []
    lanes = []

    for i_0 in range(len(lines)):
        appended = False

        if (
            i_0 in used
            or (abs(1 / slopes[i_0]) > horizontal_tol)
            or (
                lines[i_0][0][1] < vertical_threshold
                and lines[i_0][0][3] < vertical_threshold
            )
        ):
            continue

        for i_1 in range(i_0 + 1, len(lines)):
            if i_1 in used or (1 / slopes[i_0] > horizontal_tol):
                continue

            if (
                abs(1 / slopes[i_1] - 1 / slopes[i_0]) < m_tol
                and abs(intercepts[i_1] - intercepts[i_0]) < b_tol
            ) and (
                abs(intercepts[i_1] - intercepts[i_0]) > db_min
                and abs(1 / slopes[i_1] - 1 / slopes[i_0]) > dm_min
            ):
                lanes.append([lines[i_0][0], lines[i_1][0]])
                appended = True

                used.append(i_0)
                used.append(i_1)

        if not appended:
            lanes.append([lines[i_0][0]])
            used.append(i_0)

    return lanes


def draw_lanes(img, lanes, height):
    lines = []

    m_tol = 1e2
    dx_tol = 1e-3
    mul_constant = 1e6

    for pair in lanes:
        for line in pair:
            x1, y1, x2, y2 = line

            if abs(x2 - x1) < dx_tol:
                dx = (
                    max(dx_tol, (x2 - x1)) if (x2 - x1 > 0) else min(-dx_tol, (x2 - x1))
                )
            else:
                dx = x2 - x1

            m = np.clip((y2 - y1) / dx, -m_tol, m_tol)

            b_1 = x1 + ((height - y1) / m)
            b_2 = x1 + ((height / 3 - y1) / m)
            lines.append([[b_1, height, b_2, height / 3]])

    img = draw_lines(img, lines, (0, 255, 0))

    return img


def process_image(img):
    height = img.shape[0]

    lines = detect_lines(img, 5, 70, 3, 250, 100)

    if lines is None:
        return img

    lanes = detect_lanes(lines, height)

    if len(lanes) < 1:
        return img

    img = draw_lanes(img, lanes, height)

    return img
