import cv2
import numpy as np

def detect_arrow_color(image, marker_corners):
    # Unpack corners: top-left, top-right, bottom-right, bottom-left
    tl, tr, br, bl = marker_corners.reshape((4, 2))
    center = np.mean([tl, tr, br, bl], axis=0)

    # Get horizontal direction vector
    left_vec = ((tl + bl) / 2) - center
    right_vec = ((tr + br) / 2) - center

    offset = 100  # pixels from center
    box_size = 50  # box width/height

    h, w = image.shape[:2]

    def get_box(center, direction):
        if np.linalg.norm(direction) == 0:
            return None

        point = center + direction / np.linalg.norm(direction) * offset
        x, y = int(point[0]), int(point[1])

        # Clamp box to image bounds
        x1 = max(0, x - box_size // 2)
        y1 = max(0, y - box_size // 2)
        x2 = min(w, x + box_size // 2)
        y2 = min(h, y + box_size // 2)

        if x1 >= x2 or y1 >= y2:
            return None

        region = image[y1:y2, x1:x2]
        if region.size == 0:
            return None

        return region

    left_box = get_box(center, left_vec)
    right_box = get_box(center, right_vec)

    def check_color(region):
        if region is None:
            return None

        hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)

        # Red mask
        lower = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255))
        upper = cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
        red_mask = cv2.bitwise_or(lower, upper)

        # Green mask
        green_mask = cv2.inRange(hsv, (40, 70, 50), (80, 255, 255))

        red_count = cv2.countNonZero(red_mask)
        green_count = cv2.countNonZero(green_mask)

        if red_count > green_count and red_count > 30:
            return "red"
        elif green_count > red_count and green_count > 30:
            return "green"
        return None

    left_color = check_color(left_box)
    right_color = check_color(right_box)

    if left_color is not None:
        return left_color
    elif right_color is not None:
        return right_color
    else:
        return None
