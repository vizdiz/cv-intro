from lane_detection import (
    detect_lanes,
    detect_lines,
    draw_lines,
    draw_lanes,
    get_slopes_intercepts,
)


def get_lane_center(lanes, x_center, height):
    center_lane = [[x.tolist()] for x in lanes[0]]

    if len(lanes) > 1:
        for pair in lanes[1:]:
            lane = [[x.tolist()] for x in pair]

            # Debug, wrong intercept(use bottom instead of top)
            _, intercepts_0 = get_slopes_intercepts(center_lane, height)
            _, intercepts_1 = get_slopes_intercepts(lane, height)

            b_0 = sum(intercepts_0) / len(intercepts_0)
            b_1 = sum(intercepts_1) / len(intercepts_1)

            if abs(b_1 - x_center) < abs(b_0 - x_center):
                center_lane = lane

    slopes, intercepts = get_slopes_intercepts(center_lane, height)
    m, b = (sum(slopes) / len(slopes), sum(intercepts) / len(intercepts))

    # return b, m
    return center_lane


def recommend_strafe_direction(center, slope, width):
    x_center = width / 2
    x_tol = width / 10
    m_tol = 3e1

    difference = center - x_center
    strafe_direction = ""
    turn_direction = ""

    if abs(difference) < x_tol:
        strafe_direction = "forward"
    elif difference > x_tol:
        strafe_direction = "right"
    else:
        strafe_direction = "left"

    if abs(slope) > m_tol:
        turn_direction = "forward"
    elif slope < 0:
        turn_direction = "left"
    else:
        turn_direction = "right"

    return (strafe_direction, turn_direction)


def process_image(img, underwater):
    height, width, channels = img.shape

    if not underwater:
        lines = detect_lines(img, 40, 70, 5, 50, 30)
    else:
        lines = detect_lines(img, 5, 70, 3, 300, 100)

    lanes = detect_lanes(lines, height, width)

    if len(lanes) < 1:
        print("Lanes found.")
        img = draw_lines(img, lines, height)
        return img

    print("Lanes found.")

    # center_lane = get_lane_center(lanes, img.shape[0] / 2, img.shape[1])

    # img = draw_lanes(img, center_lane, height)

    # img = draw_lanes(img, lanes, height)

    b, m = get_lane_center(lanes, img.shape[0] / 2, img.shape[1])

    return b, m
