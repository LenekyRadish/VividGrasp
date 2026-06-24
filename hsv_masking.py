import cv2
import numpy as np
from PIL import Image

color_ranges = {
    'red': [
        {"lower": np.array([0, 70, 70]), "upper": np.array([5, 255, 255])},
        {"lower": np.array([175, 70, 70]), "upper": np.array([180, 255, 255])}
    ],
    'orange': [{"lower": np.array([6, 70, 70]), "upper": np.array([15, 255, 255])}],
    'yellow': [{"lower": np.array([16, 70, 70]), "upper": np.array([34, 255, 255])}],
    'green': [{"lower": np.array([35, 70, 60]), "upper": np.array([85, 255, 255])}],
    'blue': [{"lower": np.array([90, 70, 60]), "upper": np.array([130, 255, 255])}],
    'purple': [{"lower": np.array([131, 70, 50]), "upper": np.array([160, 255, 255])}]
}

def process_image_colors(image_path, output_path="color_result.png"):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not open image at {image_path}. Did you upload it?")
        return False

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    result_img = img.copy()

    for color_name, ranges in color_ranges.items():
        color_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        
        for r in ranges:
            m = cv2.inRange(hsv, r["lower"], r["upper"])
            color_mask = cv2.bitwise_or(color_mask, m)
            
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 300:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    cv2.circle(result_img, (cX, cY), 5, (0, 255, 0), -1)
                    cv2.putText(result_img, color_name.capitalize(), (cX - 25, cY - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                                
    cv2.imwrite(output_path, result_img)
    print(f"Success! Processed image saved as {output_path}")
    return True


if process_image_colors("/content/sample_data/test_image.png", "result.png"):
  display(Image.open("result.png"))
