import cv2
import numpy as np
import matplotlib.pyplot as plt

global img
# Change this variable to the desired image to scan, for example to scan image.png, just replace the stuff inside the quotes
img = cv2.imread('test_cube.jpg')


def example_contours():
    # Sample contours + mask
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lower_all = np.array([0, 80, 80])
    upper_all = np.array([179, 255, 255])
    mask_all = cv2.inRange(hsv, lower_all, upper_all)
    # Finding contours
    contours_all, _ = cv2.findContours(mask_all, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    print(f"Found {len(contours_all)} contours for all objects.")
    return contours_all

def get_shape_name(cnt):
    area = cv2.contourArea(cnt)
    peri = cv2.arcLength(cnt, True)

    if area == 0 or peri == 0:
        return "Unknown"

    # Simple approximation for polygons
    approx_simple = cv2.approxPolyDP(cnt, 0.025 * peri, True)
    vertices = len(approx_simple)

    # Detailed approximation for curved/complex shapes
    approx_detailed = cv2.approxPolyDP(cnt, 0.01 * peri, True)
    detailed_vertices = len(approx_detailed)

    x, y, w, h = cv2.boundingRect(cnt)
    aspect_ratio = w / float(h)

    circularity = 4 * np.pi * area / (peri * peri)

    hull = cv2.convexHull(cnt)
    hull_area = cv2.contourArea(hull)

    if hull_area == 0:
        return "Unknown"

    solidity = area / hull_area

    if solidity < 0.90:
        # Stars usually have more detailed points and lower solidity
        if detailed_vertices >= 9 and solidity < 0.82:
            return "Star"

        # Hearts are concave but usually more rounded/solid than stars
        if circularity > 0.55:
            return "Heart"

        return "Irregular"

    if detailed_vertices >= 12 and circularity > 0.78:
        if 0.85 <= aspect_ratio <= 1.15:
            return "Circle"
        else:
            return "Oval"

    if vertices == 3:
        return "Triangle"

    elif vertices == 4:
        if 0.85 <= aspect_ratio <= 1.15:
            return "Square"
        else:
            return "Quadrilateral"

    elif vertices == 5:
        return "Pentagon"

    elif vertices == 6:
        return "Hexagon"

    elif vertices == 7:
        return "Heptagon"

    elif vertices == 8:
        return "Octagon"

    if circularity > 0.82:
        if 0.85 <= aspect_ratio <= 1.15:
            return "Circle"
        else:
            return "Oval"

    return "Irregular"

def find_contours(contour, color):
    result_img = img.copy()
    for cnt in contour:
        area = cv2.contourArea(cnt)
        if area < 2000:
            continue
        peri = cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, 0.025 * peri, True)
        shape_name = get_shape_name(cnt)
        # Draw contour outline in black
        cv2.drawContours(result_img, [approx], -1, (0, 0, 0), 3)
        # Find center of contour to place text
        x, y, w, h = cv2.boundingRect(approx)
        cx, cy = x + w//2, y + h//2
        cv2.putText(result_img, f"{color} {shape_name}", (cx - 50, cy - 10), cv2.FONT_HERSHEY_COMPLEX, 0.4, (0, 255, 0), 2)
    return result_img


def main(): 
    contours_input = example_contours() 
    result = find_contours(contours_input, "")
    cv2.imwrite("output.png", result)
    print("Saved result as output.png")


if __name__ == "__main__":
    main()