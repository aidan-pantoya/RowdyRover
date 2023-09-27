# Rowdy Rover
# Path Finding through Python
# Aidan Pantoya, Austyn Smock, Nathan Lancaster
# September 2023

import cv2
import numpy as np

def find_dirt_path(image_path):
    image = cv2.imread(image_path)
    if image is None:
        print("Error: Cannot read image.")
        return
  
    blurred_image = cv2.GaussianBlur(image, (5,5), 0)
    hsv = cv2.cvtColor(blurred_image, cv2.COLOR_BGR2HSV)
    lower_black = np.array([0, 0, 0])
    upper_black = np.array([180, 100, 50])
    lower_red = np.array([0, 40, 40])
    upper_red = np.array([20, 255, 255])
    lower_brown = np.array([10, 40, 40])
    upper_brown = np.array([30, 200, 180])
    mask_black = cv2.inRange(hsv, lower_black, upper_black)
    mask_red = cv2.inRange(hsv, lower_red, upper_red)
    mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
    combined_mask = cv2.bitwise_or(mask_black, mask_red)
    combined_mask = cv2.bitwise_or(combined_mask, mask_brown)

    kernel = np.ones((5, 5), np.uint8)
    mask_cleaned = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
    mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask_cleaned, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)[:2]
    centroids = []
    
    for contour in sorted_contours:
        M = cv2.moments(contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])
            centroids.append((cx, cy))

    for cx, cy in centroids:
        cv2.circle(image, (cx, cy), 5, (0, 255, 0), -1) 

    if len(centroids) == 2:
        mid_x = (centroids[0][0] + centroids[1][0]) // 2
        mid_y = (centroids[0][1] + centroids[1][1]) // 2
        cv2.circle(image, (mid_x, mid_y), 5, (255, 0, 0), -1)  
        
    cv2.imshow('Original Image with Center Points', image)
    cv2.imshow('Detected Dirt Path', mask_cleaned)
    cv2.imshow('Combined', combined_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    image_path = 'C:/Users/apant/OneDrive/Pictures/CottonPOV2.jpg'
    find_dirt_path(image_path)