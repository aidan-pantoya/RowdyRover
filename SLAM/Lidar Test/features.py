import numpy as np
import math
from fractions import Fraction
from scipy.odr import *

# Landmarks
landmarks = []


class featuresDetection:
    def __init__(self):
        self.EPSILON = 10
        self.DELTA = 501
        self.SNUM = 6
        self.PMIN = 20
        self.GMAX = 20
        self.SEED_SEGMENTS = []
        self.LINE_SEGMENTS = []
        self.LASER_POINTS = []
        self.LINE_PARAMS = None
        self.NP = len(self.LASER_POINTS) - 1
        self.LMIN = 20 # Minimun length of a line segement
        self.LR = 0 # Real length of a line segment
        self.PR = 0 # The number of laser points contained in the line segment
        self.FEATURES = []

    # Euclidian distance from point1 to point2
    def dist_point2point(self, point1, point2):
        px = (point1[0] - point2[0]) ** 2
        py = (point1[1] - point2[1]) ** 2
        return math.sqrt(px + py)
    
    # Distance point to line written in the general from
    def dist_point2line(self, params, point):
        a, b, c = params
        distance = abs(a * point[0] + b * point[1] + c) / math.sqrt(a ** 2 + b ** 2)
        return distance
    
    # Extract two points from a line equation under the slope intercept form
    def line_2points(self, m, b):
        x = 5
        y = m * x + b
        x2 = 2000
        y2 = m * x2 + b
        return [(x, y), (x2, y2)]
    
    # General from to slope-intercept
    def lineForm_g2si(self, a, b, c):
        m = -a / b
        b = -c / b
        return m, b
    
    # Slope-intercept to general form
    def lineForm_si2g(self, m, b):
        a, b, c = -m, 1, -b
        if a < 0:
            a, b, c = -a, -b, -c
        den_a = Fraction(a).limit_denominator(1000).as_integer_ratio()[1]
        den_c = Fraction(c).limit_denominator(1000).as_integer_ratio()[1]

        gcd = np.gcd(den_a, den_c)
        lcm = den_a * den_c / gcd

        a = a * lcm
        b = b * lcm
        c = c * lcm
        return a, b, c
    
    def line_intersect_general(self, params1, params2):
        a1, b1, c1 = params1
        a2, b2, c2 = params2
        x = (c1 * b2 - b1 * c2) / (b1 * a2 - a1 * b2)
        y = (a1 * c2 - a2 * c1) / (b1 * a2 - a1 * b2)
        return x, y
    
    def points_2line(self, point1, point2):
        m, b = 0, 0
        if point2[0] == point1[0]:
            pass
        else:
            m = (point2[1] - point1[1]) / (point2[0] - point1[0])
            b = point2[1] - m * point2[0]
        return m, b
    
    def projection_point2line(self, point, m, b):
        x, y = point
        m2 = -1 / m
        c2 = y - m2 * x
        intersection_x = - (b - c2) / (m - m2)
        intersection_y = m2 * intersection_x + c2
        return intersection_x, intersection_y
    
    def ad2pos(self, distance, angle, robot_position):
        x = distance * math.cos(angle) + robot_position[0]
        y = -distance * math.sin(angle) + robot_position[1]
        return (int(x), int(y))
    
    def laser_points_set(self, data):
        self.LASER_POINTS = []
        if not data:
            pass
        else:
            for point in data:
                coordinates = self.ad2pos(point[0], point[1], point[2])
                self.LASER_POINTS.append([coordinates, point[1]])
        self.NP = len(self.LASER_POINTS) - 1

    # Define a function (quadratic) to fit the data with
    def linear_func(self, p, x):
        m, b = p
        return m * x + b
    
    def odr_fit(self, laser_points):
        x = np.array([i[0][0] for i in laser_points])
        y = np.array([i[0][1] for i in laser_points])

        # Create a model for fitting
        linear_model = Model(self.linear_func)

        # Create a RealData object using our initiated data from above
        data = RealData(x, y)

        # Set up ODR with the model and data
        odr_model = ODR(data, linear_model, beta0=[0., 0.])

        # Run the regression
        out = odr_model.run()
        m, b = out.beta
        return m, b
    
    def predictPoint(self, line_params, sensed_point, robotpos):
        m, b = self.points_2line(robotpos, sensed_point)
        params1 = self.lineForm_si2g(m, b)
        predx, predy = self.line_intersect_general(params1, line_params)
        return predx, predy
    
    def seed_segment_detection(self, robot_position, break_point_ind):
        flag = True
        self.NP = max(0, self.NP)
        self.SEED_SEGMENTS = []
        for i in range(break_point_ind, (self.NP - self.PMIN)):
            predicted_points_to_draw = []
            j = i + self.SNUM
            m, c = self.odr_fit(self.LASER_POINTS[i:j])

            params = self.lineForm_si2g(m, c)

            for k in range(i, j):
                predicted_point = self.predictPoint(params, self.LASER_POINTS[k][0], robot_position)
                predicted_points_to_draw.append(predicted_point)
                d1 = self.dist_point2point(predicted_point, self.LASER_POINTS[k][0])

                if d1 > self.DELTA:
                    flag = False
                    break

                d2 = self.dist_point2line(params, predicted_point)

                if d2 > self.EPSILON:
                    flag = False
                    break

            if flag:
                self.LINE_PARAMS = params
                return [self.LASER_POINTS[i:j], predicted_points_to_draw, (i, j)]
            
        return False
    
    def seed_segment_growing(self, indices, break_point):
        line_eq = self.LINE_PARAMS
        i, j = indices

        # Beginning and final points in the line segment
        pb, pf = max(break_point, i - 1), min(j + 1, len(self.LASER_POINTS) - 1)

        while self.dist_point2line(line_eq, self.LASER_POINTS[pf][0]) < self.EPSILON:
            if pf > self.NP - 1:
                break
            else:
                m, b = self.odr_fit(self.LASER_POINTS[pb:pf])
                line_eq = self.lineForm_si2g(m, b)

                point = self.LASER_POINTS[pf][0]

            pf = pf + 1
            nextpoint = self.LASER_POINTS[pf][0]
            if self.dist_point2point(point, nextpoint) > self.GMAX:
                break

        pf = pf - 1

        while self.dist_point2line(line_eq, self.LASER_POINTS[pb][0]) < self.EPSILON:
            if pb < break_point:
                break
            else:
                m, b = self.odr_fit(self.LASER_POINTS[pb:pf])
                line_eq = self.lineForm_si2g(m, b)
                point = self.LASER_POINTS[pb][0]

            pb = pb - 1
            nextpoint = self.LASER_POINTS[pb][0]
            if self.dist_point2point(point, nextpoint) > self.GMAX:
                break
        pb = pb + 1

        lr = self.dist_point2point(self.LASER_POINTS[pb][0], self.LASER_POINTS[pf][0])
        pr = len(self.LASER_POINTS[pb:pf])

        if (lr >= self.LMIN) and (pr >= self.PMIN):
            self.LINE_PARAMS = line_eq
            m, b = self.lineForm_g2si(line_eq[0], line_eq[1], line_eq[2])
            self.two_points = self.line_2points(m, b)
            self.LINE_SEGMENTS.append((self.LASER_POINTS[pb + 1][0], self.LASER_POINTS[pf - 1][0]))
            return [self.LASER_POINTS[pb:pf], self.two_points,
                    (self.LASER_POINTS[pb + 1][0], self.LASER_POINTS[pf - 1][0]), pf, line_eq, (m,b)]
        else:
            return False
        

    def lineFeats2point(self):
        new_rep = [] # New representation for the features

        for feature in self.FEATURES:
            projection = self.projection_point2line((0, 0), feature[0][0], feature[0][1])
            new_rep.append([feature[0], feature[1], projection])

        return new_rep
    
def landmark_association(landmarks):
    thresh = 10
    for l in landmarks:
        flag = False
        for i, landmark in enumerate(landmarks):
            dist = featuresDetection.dist_point2point(l[2], landmark[2])
            if dist < thresh:
                if not is_overlap(l[i], landmark[i]):
                    continue
                else:
                    landmarks.pop(i)
                    landmarks.insert(i, l)
                    flag = True
                    break
        
        if not flag:
            landmarks.append(l)

def is_overlap(seg1, seg2):
    length1 = featuresDetection.dist_point2point(seg1[0], seg1[1])
    length2 = featuresDetection.dist_point2point(seg2[0], seg2[1])
    center1 = ((seg1[0][0] + seg1[1][0]) / 2, (seg1[0][1] + seg1[1][1]) / 2)
    center2 = ((seg2[0][0] + seg2[1][0]) / 2, (seg2[0][1] + seg2[1][1]) / 2)
    dist = featuresDetection.dist_point2point(center1, center2)
    if dist > (length1 + length2) / 2:
        return False
    else:
        return True