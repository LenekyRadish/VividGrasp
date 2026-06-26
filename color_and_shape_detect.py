import cv2
import numpy as np
from PIL import Image

color_ranges = {
    'orange': [{"lower": np.array([5, 85, 150]), "upper": np.array([22, 255, 255])}],
    'green':  [{"lower": np.array([25, 45, 45]), "upper": np.array([85, 255, 255])}],
    'white':  [{"lower": np.array([0, 0, 150]), "upper": np.array([180, 50, 255])}]
}

def process_sports_vision(image_path, output_path="robot_result.png"):
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not open image at {image_path}.")
        return False

    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    result_img = img.copy()
    

    for color, ranges in color_ranges.items():
        color_mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
        for r in ranges:
            m = cv2.inRange(hsv, r["lower"], r["upper"])
            color_mask = cv2.bitwise_or(color_mask, m)
            
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        
        for cnt in contours:
            if cv2.contourArea(cnt) > 1000:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    cX = int(M["m10"] / M["m00"])
                    cY = int(M["m01"] / M["m00"])
                    
                    x, y, w, h = cv2.boundingRect(cnt)
                    if color == 'green':
                        shape_name = "Tennis"
                    elif color == 'orange':
                        shape_name = "PingPong"
                    elif color == 'white':
                        shape_name = "Baseball"
                    
                    cv2.drawContours(result_img, [cnt], -1, (0, 0, 0), 3) 
                    cv2.circle(result_img, (cX, cY), 6, (0, 0, 255), -1)    
                    
                    cx = x + (w // 2) - 35
                    cy = y - 15 if (y - 15) > 20 else y + 30
                    
                    cv2.putText(result_img, f"{color.capitalize()} {shape_name}", (cx + 1, cy + 1),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 2)
                    cv2.putText(result_img, f"{color.capitalize()} {shape_name}", (cx, cy),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 2)
                                
    cv2.imwrite(output_path, result_img)
    return True

if process_sports_vision("image.png", "result.png"):
    display(Image.open("result.png"))
