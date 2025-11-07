    import cv2
    import numpy as np

    def detect_arrow_color(image, marker_corners):
        # Unpack corners: top-left, top-right, bottom-right, bottom-left
        tl, tr, br, bl = marker_corners.reshape((4, 2))
        center = np.mean([tl, tr, br, bl], axis=0)

        # Get horizontal direction vector
        left_vec = ((tl + bl) / 2) - center
        right_vec = ((tr + br) / 2) - center

        # Define box parameters
        offset = 50  # pixels from center
        box_size = 40  # box width/height

        def get_box(center, direction):
            point = center + direction / np.linalg.norm(direction) * offset
            x, y = int(point[0]), int(point[1])
            return image[y - box_size//2:y + box_size//2, x - box_size//2:x + box_size//2]

        left_box = get_box(center, left_vec)
        right_box = get_box(center, right_vec)

        def check_color(region):
            hsv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
            red_mask = cv2.inRange(hsv, (0, 70, 50), (10, 255, 255)) + cv2.inRange(hsv, (170, 70, 50), (180, 255, 255))
            green_mask = cv2.inRange(hsv, (40, 70, 50), (80, 255, 255))
            red_count = cv2.countNonZero(red_mask)
            green_count = cv2.countNonZero(green_mask)
            if red_count > green_count and red_count > 30:
                return "red"
            elif green_count > red_count and green_count > 30:
                return "green"
            return "none"

        left_color = check_color(left_box)
        right_color = check_color(right_box)

        if left_color != "none":
            return f"left-{left_color}"
        elif right_color != "none":
            return f"right-{right_color}"
        else:
            return "none"