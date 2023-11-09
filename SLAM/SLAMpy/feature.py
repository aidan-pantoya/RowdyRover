import cv2

# Initialize ORB detector
orb = cv2.ORB_create()

# Dummy RGB and depth frames (replace these with actual frames from your camera)
rgb_frame1 = cv2.imread('frame1_rgb.png', cv2.IMREAD_COLOR)
depth_frame1 = cv2.imread('frame1_depth.png', cv2.IMREAD_GRAYSCALE)

rgb_frame2 = cv2.imread('frame2_rgb.png', cv2.IMREAD_COLOR)
depth_frame2 = cv2.imread('frame2_depth.png', cv2.IMREAD_GRAYSCALE)

# Find keypoints and descriptors in the images
keypoints1, descriptors1 = orb.detectAndCompute(rgb_frame1, None)
keypoints2, descriptors2 = orb.detectAndCompute(rgb_frame2, None)

# Create a BFMatcher (Brute Force Matcher) object
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

# Match descriptors using KNN
matches = bf.match(descriptors1, descriptors2)

# Sort the matches based on their distances
matches = sorted(matches, key=lambda x: x.distance)

# Draw the matches
matching_result = cv2.drawMatches(rgb_frame1, keypoints1, rgb_frame2, keypoints2, matches[:10], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# Display the matching result
cv2.imshow('Feature Matches', matching_result)
cv2.waitKey(0)
cv2.destroyAllWindows()
