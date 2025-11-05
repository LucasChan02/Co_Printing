import cv2
import numpy as np

# --- Parameters for Contour Processing ---
# Factor for approximating contour shape. Lower value = more detail, higher = more simplification.
APPROX_EPSILON_FACTOR = 0.0035
# Thresholds for segment length (in pixels)
MIN_SEGMENT_LEN = 60
MAX_SEGMENT_LEN = 600
# Physical measurement conversion
PIXEL_TO_MICRON = 1.13636  # Microns per pixel

# 1. Load Image
img = cv2.imread("../Micro/1103_0001.png")
if img is None:
    print("Error: Image not found.")
    exit()

# Create output images
output_img_approx = img.copy()
# Create a black image to draw the processed lines on for clarity
processed_lines_img = np.zeros_like(img)

# 2. Convert to HSV and create mask
hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
lower_green = np.array([40, 40, 40])
upper_green = np.array([70, 255, 255])
mask_green = cv2.inRange(hsv, lower_green, upper_green)

# 3. Clean the mask
kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

# 4. Find green contours
contours, _ = cv2.findContours(mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

# 5. Process each contour
total_length_pixels = 0
for cnt in contours:
    # Approximate the contour to a polygon
    epsilon = APPROX_EPSILON_FACTOR * cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    # Draw the full approximated polygon on the original image
    cv2.polylines(
        output_img_approx, [approx], isClosed=True, color=(255, 0, 0), thickness=2
    )

    # Analyze and draw segments of the approximated polygon
    for i in range(len(approx)):
        p1 = approx[i][0]
        p2 = approx[(i + 1) % len(approx)][0]  # Next point with wrap-around

        segment_length = np.linalg.norm(p1 - p2)

        # Check if segment length is within the defined thresholds
        if MIN_SEGMENT_LEN <= segment_length <= MAX_SEGMENT_LEN:
            total_length_pixels += segment_length
            # Draw this approved segment on the black image
            cv2.line(
                processed_lines_img, tuple(p1), tuple(p2), (0, 255, 0), 2
            )  # Green for approved
        else:
            # Draw rejected segments in red for visualization
            cv2.line(
                processed_lines_img, tuple(p1), tuple(p2), (0, 0, 255), 2
            )  # Red for rejected


# 6. Convert length to physical units and print
length_microns = total_length_pixels * PIXEL_TO_MICRON
print(f"Total length of approved segments ≈ {length_microns / 1000:.4f} mm")


# 7. Display the results
# Resize
cv2.namedWindow("Approximated Contour", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Approximated Contour", 800, 600)
cv2.namedWindow("Processed Segments", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Processed Segments", 800, 600)

cv2.imshow("Approximated Contour", output_img_approx)
cv2.imshow("Processed Segments", processed_lines_img)
cv2.waitKey(0)
cv2.destroyAllWindows()
