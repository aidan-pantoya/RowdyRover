# Rowdy Rover
# Path Finding through Python
# Aidan Pantoya, Austyn Smock, Nathan Lancaster
# September 2023
import sys
import cv2
import numpy as np
import OpenEXR
import Imath
#path = 'C:/Users/apant/Downloads/DirtyField1.jpg' 
#path = 'C:/Users/apant/OneDrive/Desktop/Capstone/PaloDuroPath2.mp4'

target_percent = 0
start_percent = target_percent

def extract_pixel_values(image):
    height, width = image.shape[:2]
    left_point = (width // 4, height - int(height*0.25))
    right_point = (3 * width // 4, height - int(height*0.25))
    region_width = int(0.05 * width)
    region_height = int(0.1 * height)
    shift_value = int(0.4 * width)
    
    def get_five_points(center, rw, rh,shift):
        return [
            (center[0] - rw // 2 + shift, center[1] - rh // 2),  # Top-left
            (center[0] + rw // 2 + shift, center[1] - rh // 2),  # Top-right
            (center[0] - rw // 2 + shift, center[1] + rh // 2),  # Bottom-left
            (center[0] + rw // 2 + shift, center[1] + rh // 2),  # Bottom-right
            (center[0] + shift, center[1]),   
        ]
    left_points = get_five_points(left_point, region_width, region_height,shift_value)
    right_points = get_five_points(right_point, region_width, region_height,-shift_value)
    left_values = [image[y, x] for x, y in left_points]
    right_values = [image[y, x] for x, y in right_points]
    display_image = image.copy()
    for x, y in left_points + right_points:
        cv2.circle(display_image, (x, y), 2, (0, 0, 255), -1)
    cv2.imshow('Extracted Points', display_image)
    cv2.waitKey(15)
    return left_values, right_values

def mask_top_corners(image):
    height, width = image.shape[:2]
    mask = np.ones((height, width), dtype=np.uint8) * 255
    # corner_width = int(0.4 * width)
    # triangle1 = np.array([(0, 0), (corner_width, 0), (0, corner_width)])
    # triangle2 = np.array([(width, 0), (width - corner_width, 0), (width, corner_width)])
    # cv2.drawContours(mask, [triangle1], 0, 0, -1)
    # cv2.drawContours(mask, [triangle2], 0, 0, -1)
    top_height = int(0.2 * height)
    cv2.rectangle(mask, (0, 0), (width, top_height), 0, -1)
    bottom_height = int(0.9 * height)
    cv2.rectangle(mask, (0, bottom_height), (width, height), 0, -1)
    return mask

def process_contours(image_section, mask_section):
    contours, _ = cv2.findContours(mask_section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []
    all_points = []
    for contour in contours:
        if cv2.contourArea(contour) > 0:
            M = cv2.moments(contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = contour[0][0]
            centroids.append((cx, cy))
            for point in contour:
                all_points.append(tuple(point[0]))
    center = 0
    try:
        min_x = min(pt[0] for pt in all_points)
        max_x = max(pt[0] for pt in all_points)
        min_y = min(pt[1] for pt in all_points)
        max_y = max(pt[1] for pt in all_points)
        center_x = (min_x + max_x) / 2
        center_y = (min_y + max_y) / 2
        center = (int(center_x), int(center_y))
    except:
        center = 0
    return image_section, center, all_points

def determine_color(target_percent, image, left_pixel, right_pixel):
    total_pixels = image.shape[0] * image.shape[1]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    combined_mask = np.zeros_like(image[:,:,0]) 
    for r in left_pixel[0]:
        for g in left_pixel[1]:
            for b in left_pixel[2]:
                for rr in right_pixel[0]:
                    for gg in right_pixel[1]:
                        for bb in right_pixel[2]:
                            temp_left_pixel = [r, g, b]
                            temp_right_pixel = [rr, gg, bb]
                            hsv_left_pixel = cv2.cvtColor(np.uint8([[temp_left_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
                            hsv_right_pixel = cv2.cvtColor(np.uint8([[temp_right_pixel]]), cv2.COLOR_BGR2HSV)[0][0]
                            hsv_mean = np.mean([hsv_left_pixel, hsv_right_pixel], axis=0)
                            lower_bound = np.array([hsv_mean[0] - int(target_percent/2), hsv_mean[1] - int(target_percent), hsv_mean[2] - int(target_percent)])
                            upper_bound = np.array([hsv_mean[0] + int(target_percent/2), hsv_mean[1] + int(target_percent), hsv_mean[2] + int(target_percent)])
                            mask = cv2.inRange(hsv, lower_bound, upper_bound)
                            combined_mask = cv2.bitwise_or(combined_mask, mask)
    corner_mask = mask_top_corners(image) 
    final_mask = cv2.bitwise_and(combined_mask, corner_mask)
    kernel = np.ones((11, 11), np.uint8)
    mask_cleaned = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
    mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_CLOSE, kernel)
    processed_image, center, Collect = process_contours(image, mask_cleaned)  
    return processed_image, center, Collect, mask_cleaned, total_pixels, final_mask

def clamp(value, min_value, max_value):
    return max(min(value, max_value), min_value)

def resize_image_to_screen(image, screen_width=600, screen_height = 300):
    img_height, img_width = image.shape[:2]
    scale = min(screen_width / img_width, screen_height / img_height)
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

def process_image(image):
    global target_percent
    global start_percent
    target_percent = start_percent
    left_pixel, right_pixel = extract_pixel_values(image)
    ranger = 0.08
    result_image, cent_avg, Collect, mask_cleaned, total_pix, final_mask = determine_color(target_percent, image, left_pixel, right_pixel)
    while cent_avg == 0 or np.sum(final_mask) / 255 <= total_pix * ranger:
        target_percent += 1
        result_image, cent_avg, Collect, mask_cleaned, total_pix, final_mask = determine_color(target_percent, image, left_pixel, right_pixel)
    centx, _ = cent_avg
    cv2.imshow('mask img', final_mask)
    cv2.waitKey(100)
    r_o_l = result_image.shape[1] - centx - (result_image.shape[1] / 2)
    Collect = sorted(Collect, key=lambda x: x[1])
    _, img_width = result_image.shape[:2]
    c_sz = max(int(img_width * 0.007), 3)
    for c in Collect:
        result_image = cv2.circle(result_image, c, c_sz, (155, 40, 0), -1)
    result_image = cv2.circle(result_image, cent_avg, c_sz, (0, 0, 255), -1)
    result_image = cv2.circle(result_image, (result_image.shape[1]//2, result_image.shape[0]//2), c_sz//2, (0, 157, 254), -1)
    cv2.imshow('result img', result_image)
    cv2.waitKey(100)
    return r_o_l

def handle_input(path):
    if path.endswith(('.jpg', '.png')):
        image = cv2.imread(path)
        if image is None:
            print("Error: Cannot read image.")
            return
        image = resize_image_to_screen(image,600,300)
        image = equal_hist(image)
        r_o_l = process_image(image)
    elif path.endswith('.mp4'):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
             print("Error: Couldn't open video.")
             return
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            frame = resize_image_to_screen(frame,600,300)
            frame = equal_hist(frame)
            r_o_l = process_image(frame)
    r_o_l = r_o_l * 90 / 600
    if r_o_l > 0:
        print("Go Left degs:", r_o_l)
    elif r_o_l < 0:
        print('Go Right degs:', r_o_l)
    else:
        print("Go Straight")
    return -r_o_l


def equal_hist(image):
    _, img_width = image.shape[:2]
    k = int(img_width * 0.01)
    if k % 2 == 0:
        k +=1
    image = cv2.blur(image,(k,k))
    ycrcb = cv2.cvtColor(image, cv2.COLOR_BGR2YCrCb)
    y, cr, cb = cv2.split(ycrcb)
    y_eq = cv2.equalizeHist(y)
    merged = cv2.merge([y_eq, cr, cb])
    equ_bgr = cv2.cvtColor(merged, cv2.COLOR_YCrCb2BGR)
    return equ_bgr

#handle_input(path)