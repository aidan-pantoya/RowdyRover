import concurrent.futures
import cv2
import math
import numpy as np
from ultralytics import YOLO


cap = cv2.VideoCapture(0)
cap.set(3, 640)  
cap.set(4, 480) 

executor = concurrent.futures.ThreadPoolExecutor(max_workers=2) 

model = YOLO("C:/Users/apant/Downloads/yolo8/yolov8n.pt")
# model = YOLO("C:/Users/apant/Downloads/yolo8/yolov8l.pt") 
# model = YOLO("C:/Users/apant/Downloads/yolo8/yolov8s.pt") 

classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]


class Node:

    def __init__(self, parent=None, position=None):
        self.parent = parent
        self.position = position
        self.g = 0
        self.h = 0
        self.f = 0

    def __eq__(self, other):
        return self.position == other.position

def astar(maze, start, end):
    start_node = Node(None, start)
    end_node = Node(None, end)

    open_list = [start_node]
    closed_list = []

    while open_list:
        current_node = min(open_list, key=lambda x: x.f)
        open_list.remove(current_node)
        closed_list.append(current_node)

        if current_node == end_node:
            path = []
            while current_node:
                path.append(current_node.position)
                current_node = current_node.parent
            return path[::-1]

        children = []
        for new_position in [(-1, -1), (-1, 0), (-1, 1), (0, -1), (0, 1), (1, -1), (1, 0), (1, 1)]:  
            node_position = (current_node.position[0] + new_position[0], current_node.position[1] + new_position[1])

            if node_position[0] > (len(maze) - 1) or node_position[0] < 0 or node_position[1] > (len(maze[0]) - 1) or node_position[1] < 0:
                continue

            if maze[node_position[1]][node_position[0]] != 0:  
                continue

            new_node = Node(current_node, node_position)
            children.append(new_node)

        for child in children:
            if child in closed_list:
                continue

            child.g = current_node.g + 1
            child.h = ((child.position[0] - end_node.position[0]) ** 2) + ((child.position[1] - end_node.position[1]) ** 2)
            child.f = child.g + child.h

            if any(open_node for open_node in open_list if child == open_node and child.g > open_node.g):
                continue

            open_list.append(child)


def create_maze(img, boxes):
    grid_size = 100 
    scale = max(img.shape[0], img.shape[1]) // grid_size
    print(f'scale: {scale}')
    maze = np.zeros((grid_size, grid_size), dtype=np.int8)
    
    for box in boxes:
        x1, y1, x2, y2 = [int(coord / scale) for coord in box.xyxy[0]]
        maze[y1:y2+1, x1:x2+1] = 1 
    return maze, scale

def process_image_and_find_path(img):
    results = model(img, stream=True)   

    height, width = img.shape[:2]

    x_start, y_start = width // 10, height // 10
    x_end, y_end = width - width // 10, height - height // 10

    obstacles = [box for r in results for box in r.boxes if (x_start <= box.xyxy[0][0] <= x_end and y_start \
        <= box.xyxy[0][1] <= y_end and x_start <= box.xyxy[0][2] <= x_end and y_start <= box.xyxy[0][3] <= y_end)]

    maze, scale = create_maze(img, obstacles)
    start = (img.shape[1] // 2 // scale, img.shape[0] // scale - 1)
    end = (img.shape[1] // 2 // scale, 0)
    path = astar(maze, start, end)
    
    if path:
        for point in path:
            img_point = (point[0] * scale, point[1] * scale)
            cv2.circle(img, img_point, 5, (90, 50, 205), -1)

    for box in obstacles:
        x1, y1, x2, y2 = box.xyxy[0]
        x1, y1, x2, y2 = map(int, [x1, y1, x2, y2])
        cv2.rectangle(img, (x1, y1), (x2, y2), (5, 0, 55), 3)

        confidence = math.ceil(box.conf[0] * 100) / 100
        cls = int(box.cls[0])

        org = (x1, y1)
        cv2.putText(img, f"{classNames[cls]}: {confidence}", org, cv2.FONT_HERSHEY_TRIPLEX, 1, (255, 205, 200), 1)

    return img


def main_loop():
    while True:
        success, img = cap.read()
        if not success:
            break
        img = cv2.resize(img, (300, 200))

        future = executor.submit(process_image_and_find_path, img)
        try:
            img = future.result(timeout=2)

        except concurrent.futures.TimeoutError:
            print("Took too long... moving on...")
            continue  

        img = cv2.resize(img, (600, 400))
        cv2.imshow('Obstacles + Path', img)
        if cv2.waitKey(1) == ord('q'):
            break

if __name__ == "__main__":
    try:
        main_loop()
    finally:
        cap.release()
        cv2.destroyAllWindows()
        executor.shutdown(wait=True)  