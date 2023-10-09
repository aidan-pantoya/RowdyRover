# Rowdy Rover
# Path Finding through Python
# Aidan Pantoya, Austyn Smock, Nathan Lancaster
# September 2023

import cv2
import numpy as np

#path = 'C:/Users/apant/Downloads/DirtyField1.jpg' 
path = 'C:/Users/apant/OneDrive/Desktop/Capstone/PaloDuroPath3.mp4'

def mask_top_corners(image):
    height, width = image.shape[:2]
    mask = np.ones((height, width), dtype=np.uint8) * 255

    # Masking top corners
    corner_width = int(0.55 * width)
    triangle1 = np.array([(0, 0), (corner_width, 0), (0, corner_width)])
    triangle2 = np.array([(width, 0), (width - corner_width, 0), (width, corner_width)])
    cv2.drawContours(mask, [triangle1], 0, 0, -1)
    cv2.drawContours(mask, [triangle2], 0, 0, -1)
    
    # Masking top and bottom
    top_height = int(0.15 * height)
    cv2.rectangle(mask, (0, 0), (width, top_height), 0, -1)
    bottom_height = int(0.7 * height)
    cv2.rectangle(mask, (0, bottom_height), (width, height), 0, -1)

    # Masking 25% on each side
    left_width = int(0.25 * width)
    cv2.rectangle(mask, (0, 0), (left_width, height), 0, -1)
    right_width = width - left_width
    cv2.rectangle(mask, (right_width, 0), (width, height), 0, -1)

    return mask

def process_contours(image_section, mask_section):
    #mask_section = cv2.blur(mask_section, (10, 10))
    Collect = []
    Collecty = []
    contours, _ = cv2.findContours(mask_section, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    centroids = []

    for contour in contours:
        if cv2.contourArea(contour) > 0:
            M = cv2.moments(contour)
            if M["m00"] != 0:  
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
            else:
                cx, cy = contour[0][0]
            centroids.append((cx, cy))
            Collect.append((cx,cy))
            Collecty.append(cy)
            cv2.circle(image_section, (cx, cy), 5, (0, 0, 255), -1) 
            if cv2.contourArea(contour) > 1000:
                centroids.append((cx, cy))
    avg = 0
    cnt = 0
    try:
        for x in range(0,len(centroids)):
            avg  += centroids[x][0]
            cnt +=1
        center_x = int(avg / cnt)
        cv2.circle(image_section, (center_x, 1500), 20, (150, 0, 205), -1)
    except:
        center_x = 0
        cv2.circle(image_section, (center_x, 1500), 20, (150, 0, 205), -1)
    return image_section, center_x, Collect

def determine_color(target_percent, image):
    total_pixels = image.shape[0] * image.shape[1]
    target_pixels = int(target_percent * total_pixels)
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_red_orange = np.array([0, 40, 40])
    upper_red_orange = np.array([20, 255, 255])
    lower_brown = np.array([10, 40, 40])
    upper_brown = np.array([30, 255, 255])

    lower_green = np.array([35, 40, 40])
    upper_green = np.array([85, 255, 255])
    lower_yellow = np.array([20, 40, 200])
    upper_yellow = np.array([35, 255, 255])
    lower_white = np.array([0, 0, 200])
    upper_white = np.array([180, 40, 255])

    total_pixels = image.shape[0] * image.shape[1]
    for _ in range(100):  
        mask_red_orange = cv2.inRange(hsv, lower_red_orange, upper_red_orange)
        mask_brown = cv2.inRange(hsv, lower_brown, upper_brown)
        combined_path_mask = cv2.bitwise_or(mask_red_orange, mask_brown)

        mask_green = cv2.inRange(hsv, lower_green, upper_green)
        mask_yellow = cv2.inRange(hsv, lower_yellow, upper_yellow)
        mask_white = cv2.inRange(hsv, lower_white, upper_white)
        combined_plant_mask = cv2.bitwise_or(mask_green, mask_yellow)
        combined_plant_mask = cv2.bitwise_or(combined_plant_mask, mask_white)        
        final_mask = cv2.bitwise_and(combined_path_mask, cv2.bitwise_not(combined_plant_mask))

        corner_mask = mask_top_corners(image)
        final_mask = cv2.bitwise_and(final_mask, corner_mask)
        
        if np.sum(final_mask) / 255 <= target_pixels:
            break

        upper_red_orange[0] -= 1
        upper_brown[0] -= 1

    kernel = np.ones((11, 11), np.uint8)
    mask_cleaned = cv2.morphologyEx(final_mask, cv2.MORPH_OPEN, kernel)
    mask_cleaned = cv2.morphologyEx(mask_cleaned, cv2.MORPH_CLOSE, kernel)

    width_half = image.shape[1] // 2
    image_left, image_right = image[:, :width_half], image[:, width_half:]
    mask_left, mask_right = mask_cleaned[:, :width_half], mask_cleaned[:, width_half:]

    image_left, cent1, Collect1 = process_contours(image_left, mask_left)
    image_right, cent2, Collect2 = process_contours(image_right, mask_right)
    
    return image_left, image_right, cent1, cent2, Collect1, Collect2, mask_cleaned, width_half


def process_image(image):
    global target_percent 
    if target_percent - 0.01 > 0.035:
        target_percent = target_percent - 0.01
    cent1 = 0
    cent2 = 0
    image_left, image_right, cent1, cent2, Collect1, Collect2, mask_cleaned,width_half = determine_color(target_percent, image)
    while (cent1 == 0 or cent2 == 0) and (target_percent < 0.25):
        target_percent += 0.005
        print("TP: "+str(target_percent))
        image_left, image_right, cent1, cent2, Collect1, Collect2, mask_cleaned,width_half = determine_color(target_percent, image)
        
    cent_avg = int((cent1 + cent2 + width_half) / 2)
    result_image = np.concatenate((image_left, image_right), axis=1)
    height, width = result_image.shape[:2]

    r_o_l = (width - cent_avg) - (width / 2)
    Collect1 = sorted(Collect1, key=lambda x: x[1])
    Collect2 = sorted(Collect2, key=lambda x: x[1])

    c1sz = int(len(Collect1) / 2)
    c2sz = int(len(Collect2) / 2)

    clct11 = Collect1[0:c1sz]
    clct12 = Collect1[c1sz:2 * c1sz]
    clct21 = Collect2[0:c2sz]
    clct22 = Collect2[c2sz:2 * c2sz]

    cl1 = [clct11, clct12]
    cl2 = [clct21, clct22]
    svy = []
    for c1, c2 in zip(cl1, cl2):
        x1 = sum(t[0] for t in c1)
        y1 = sum(t[1] for t in c1)
        x2 = sum(t[0] for t in c2)
        y2 = sum(t[1] for t in c2)

        if len(c1) > 0:
            x1 /= len(c1)
            y1 /= len(c1)
        if len(c2) > 0:
            x2 /= len(c2)
            y2 /= len(c2)

        chx = int((x1 + x2 + width / 2) / 2)
        chy = int((y1 + y2) / 2)
        cv2.circle(result_image, (chx, chy), 25, (155, 40, 0), -1)
        cv2.circle(result_image, (int(x1), int(y1)), 25, (155, 40, 0), -1)
        cv2.circle(result_image, (int(x2 + width / 2), int(y2)), 25, (155, 40, 0), -1)
        cv2.line(result_image, (int(x1), int(y1)), (int(x2 + width / 2), int(y2)), (155, 40, 0), 2)
        svy.append(y1)
        svy.append(y2)
    if r_o_l > 0:
        print("Go Left: "+str(r_o_l))
    elif r_o_l < 0:
        print('Go Right: '+str(r_o_l))
    else:
        print("Go Straight")
    
    cv2.circle(result_image, (cent_avg, int(np.mean(svy))), 50, (0, 0, 255), -1) 
    height, width = result_image.shape[:2]
    cv2.circle(result_image, (int(width/2), int(height/2)), 30, (0, 157, 254), -1)
    return result_image, mask_cleaned

def resize_image_to_screen(image, screen_width, screen_height):
    img_height, img_width = image.shape[:2]
    scale = min(screen_width / img_width, screen_height / img_height)
    return cv2.resize(image, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)

def handle_image_or_video(path):
    if path.endswith('.jpg'):
        image = cv2.imread(path)
        if image is None:
            print("Error: Cannot read image.")
            return
        processed_image, mask_cleaned = process_image(image)
        processed_image = resize_image_to_screen(processed_image,600,500)
        mask_cleaned = resize_image_to_screen(mask_cleaned,600,500)
        cv2.imshow('Original Image with Center Points', processed_image)
        cv2.imshow('Detected Dirt Path', mask_cleaned)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
    
    elif path.endswith('.mp4'):
        cap = cv2.VideoCapture(path)
        if not cap.isOpened():
             print("Error: Couldn't open video.")
             return
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            processed_frame, mask_cleaned = process_image(frame)
            processed_frame = resize_image_to_screen(processed_frame,600,500)
            mask_cleaned = resize_image_to_screen(mask_cleaned,600,500)
            cv2.imshow('Original Frame with Center Points', processed_frame)
            cv2.imshow('Detected Dirt Path', mask_cleaned)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
        cap.release()
        cv2.destroyAllWindows()

target_percent = 0.035
handle_image_or_video(path)

