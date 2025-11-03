import cv2

# Load image in color
img = cv2.imread("../Micro/1103_0001.png")

# Split the image into its B, G, R channels and select the green channel
r, g, b = cv2.split(img)

# Apply Gaussian Blur to reduce noise
blur = cv2.GaussianBlur(g, (5, 5), 0.8)

# Apply Canny Edge Detector
edges = cv2.Canny(blur, threshold1=20, threshold2=60)

# Display result
cv2.imshow("Canny Edge Detection", edges)

# Save the result
cv2.imwrite("canny_edges.png", edges)

cv2.waitKey(0)
cv2.destroyAllWindows()
