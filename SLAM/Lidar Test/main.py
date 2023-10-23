from env import *
from sensors import *
from features import *
import random
import pygame

# Change this to false if you don't want to run the lidar point section of code
lidarBool = False
featureBool = False
mappingBool = True

def random_color():
    levels = range(32, 256, 32)
    return tuple(random.choice(levels) for _ in range(3))

# Lidar points code
if lidarBool:
    environment = buildEnvironment((600,1200))
    environment.originalMap = environment.map.copy()
    laser = laserSensor(200, environment.originalMap, uncertainty=(0.5, 0.01))
    environment.map.fill((0, 0, 0))
    environment.infomap = environment.map.copy()

    running = True

    while running:
        sensorOn = False
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensorOn = True
            elif not pygame.mouse.get_focused():
                sensorOn = False
        if sensorOn:
            position = pygame.mouse.get_pos()
            laser.position = position
            sensor_data = laser.sense_obstacles()
            environment.dataStorage(sensor_data)
            environment.show_sensorData()

        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()

# Feature detection code
if featureBool:
    featureMap = featuresDetection()
    environment = buildEnvironment((600,1200))
    environment.originalMap = environment.map.copy()
    laser = laserSensor(200, environment.originalMap, uncertainty=(0.5, 0.01))
    environment.map.fill((255, 255, 255))
    environment.infomap = environment.map.copy()
    originalMap = environment.map.copy()

    running = True
    feature_detection = True
    break_point_ind = 0

    while running:
        environment.infomap = originalMap.copy()
        feature_detection = True
        break_point_ind = 0
        endpoints = [0, 0]
        sensorOn = False
        predicted_points_to_draw = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensorOn = True
            elif not pygame.mouse.get_focused():
                sensorOn = False
        if sensorOn:
            position = pygame.mouse.get_pos()
            laser.position = position
            sensor_data = laser.sense_obstacles()
            featureMap.laser_points_set(sensor_data)
            while break_point_ind < (featureMap.NP - featureMap.PMIN):
                seedSeg = featureMap.seed_segment_detection(laser.position, break_point_ind)
                if seedSeg == False:
                    break
                else:
                    seedSegment = seedSeg[0]
                    predicted_points_to_draw = seedSeg[1]
                    indices = seedSeg[2]
                    results = featureMap.seed_segment_growing(indices, break_point_ind)

                    if results == False:
                        break_point_ind = indices[1]
                        continue
                    else:
                        line_eq = results[1]
                        m, c = results[5]
                        line_seg = results[0]
                        outermost = results[2]
                        break_point_ind = results[3]

                        endpoints[0] = featureMap.projection_point2line(outermost[0], m, c)
                        endpoints[1] = featureMap.projection_point2line(outermost[1], m, c)

                        color = random_color()
                        for point in line_seg:
                            environment.infomap.set_at((int(point[0][0]), int(point[0][1])), (0, 255, 0))
                            pygame.draw.circle(environment.infomap, color, (int(point[0][0]), int(point[0][1])), 2, 0)
                        pygame.draw.line(environment.infomap, (255, 0, 0), endpoints[0], endpoints[1], 2)

                        environment.dataStorage(sensor_data)                                 

        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()

# Creating map
if mappingBool:
    featureMap = featuresDetection()
    environment = buildEnvironment((600,1200))
    environment.originalMap = environment.map.copy()
    laser = laserSensor(200, environment.originalMap, uncertainty=(0.5, 0.01))
    environment.map.fill((255, 255, 255))
    environment.infomap = environment.map.copy()
    originalMap = environment.map.copy()

    running = True
    feature_detection = True
    break_point_ind = 0

    while running:
        environment.infomap = originalMap.copy()
        feature_detection = True
        break_point_ind = 0
        endpoints = [0, 0]
        sensorOn = False
        predicted_points_to_draw = []

        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if pygame.mouse.get_focused():
                sensorOn = True
            elif not pygame.mouse.get_focused():
                sensorOn = False
        if sensorOn:
            position = pygame.mouse.get_pos()
            laser.position = position
            sensor_data = laser.sense_obstacles()
            featureMap.laser_points_set(sensor_data)
            while break_point_ind < (featureMap.NP - featureMap.PMIN):
                seedSeg = featureMap.seed_segment_detection(laser.position, break_point_ind)
                if seedSeg == False:
                    break
                else:
                    seedSegment = seedSeg[0]
                    predicted_points_to_draw = seedSeg[1]
                    indices = seedSeg[2]
                    results = featureMap.seed_segment_growing(indices, break_point_ind)

                    if results == False:
                        break_point_ind = indices[1]
                        continue
                    else:
                        line_eq = results[1]
                        m, c = results[5]
                        line_seg = results[0]
                        outermost = results[2]
                        break_point_ind = results[3]

                        endpoints[0] = featureMap.projection_point2line(outermost[0], m, c)
                        endpoints[1] = featureMap.projection_point2line(outermost[1], m, c)

                        featureMap.FEATURES.append([[m, c], endpoints])
                        pygame.draw.line(environment.infomap, (0, 255, 0), endpoints[0], endpoints[1], 1)
                        environment.dataStorage(sensor_data)

                        featureMap.FEATURES = featureMap.lineFeats2point()
                        features = featuresDetection()
                        features.landmark_association(featureMap.FEATURES)

            for landmark in landmarks:
                pygame.draw.line(environment.infomap, (0, 0, 255), landmark[1][0], landmark[1][1], 2)                               

        environment.map.blit(environment.infomap, (0, 0))
        pygame.display.update()