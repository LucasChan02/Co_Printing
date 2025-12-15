import argparse
import os
import math
import numpy as np
import cv2
from boundary_detection_rgb import contact_boundary_detection_rgb

# --- Unit Conversion Constants ---
PIXEL_TO_MICRON = 1.13636
MICRON_TO_MM = 0.001
PIXEL_TO_MM = PIXEL_TO_MICRON * MICRON_TO_MM

# --- Category Constants ---
# Defined based on Strength-aware_outline_draft.txt numbering
CAT_HORIZONTAL = 1      # Horizontal_Interlock_Surface
CAT_CLOSING = 2         # Adhesion_Interface_Closing
CAT_OPENING_UPPER = 3   # Adhesion_Interface_Opening_Upper
CAT_OPENING_LOWER = 4   # Adhesion_Interface_Opening_Lower
CAT_UNCLASSIFIED = 0    # Fallback

def calculate_angle(p1, p2):
    """
    Calculates the angle of the segment (p1 -> p2) relative to the horizontal.
    
    Returns:
        float: Angle in degrees.
    """
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    
    theta_rad = math.atan2(dy, dx)
    theta_deg = math.degrees(theta_rad)
    
    # Adjust for "Left-Down" segments (dx < 0) to map to the correct negative angle range
    # required for the "Opening Lower" classification (-45 to -10 degrees).
    # A standard atan2 result for this quadrant is (90, 180], so subtracting 180 maps it to (-90, 0].
    if dx < 0:
        theta_deg -= 180
        
    return theta_deg

def classify_segment(p1, p2, theta_h=10, theta_c=45):
    """
    Classifies a segment based on its angle relative to horizontal.
    
    Returns:
        int: Category ID (see constants).
        float: Angle.
    """
    angle = calculate_angle(p1, p2)
    
    if abs(angle) <= theta_h:
        return CAT_HORIZONTAL, angle
    elif abs(angle) > theta_c:
        return CAT_CLOSING, angle
    elif theta_h < angle <= theta_c:
        return CAT_OPENING_UPPER, angle
    elif -theta_c <= angle < -theta_h:
        return CAT_OPENING_LOWER, angle
    else:
        return CAT_UNCLASSIFIED, angle

def calculate_polygon_area(vertices):
    """Calculates the area of a polygon using the Shoelace formula."""
    n = len(vertices)
    area = 0.0
    for i in range(n):
        j = (i + 1) % n
        area += vertices[i][0] * vertices[j][1]
        area -= vertices[j][0] * vertices[i][1]
    return abs(area) / 2.0

def calculate_vir(segments, mid, l_projected):
    """
    Calculates Volumetric Interlock Ratio (VIR).
    VIR = Area_interlock / (L_projected * MID)
    
    Method:
        Calculates the area between the polyline and a vertical baseline (x = mean_x)
        using trapezoidal integration. This approximates the "geometric overlap" 
        or interlock area.
    """
    if not segments:
        return 0.0
        
    # Construct polyline vertices
    polyline = [segments[0][0]]
    for _, p2 in segments:
        polyline.append(p2)
    
    points = np.array(polyline)
    x_coords = points[:, 0]
    
    # Define Baseline as the mean X coordinate
    x_base = np.mean(x_coords)
    
    total_interlock_area = 0.0
    for p1, p2 in segments:
        x1, y1 = p1
        x2, y2 = p2
        
        # Check if segment crosses the baseline
        if (x1 - x_base) * (x2 - x_base) < 0:
            # Find intersection point
            t = (x_base - x1) / (x2 - x1)
            y_int = y1 + t * (y2 - y1)
            
            # Calculate area of two sub-trapezoids (one on each side of baseline)
            h1 = abs(x1 - x_base)
            h_int = 0
            dy1 = y_int - y1
            area1 = 0.5 * (h1 + h_int) * dy1
            
            h2 = abs(x2 - x_base)
            dy2 = y2 - y_int
            area2 = 0.5 * (h_int + h2) * dy2
            
            total_interlock_area += area1 + area2
        else:
            # Standard trapezoid area relative to baseline
            h1 = abs(x1 - x_base)
            h2 = abs(x2 - x_base)
            dy = abs(y2 - y1)
            total_interlock_area += 0.5 * (h1 + h2) * dy
            
    bbox_area = l_projected * mid
    if bbox_area == 0:
        return 0.0
        
    return total_interlock_area / bbox_area

def analyze_image(image_path, visualize=True):
    # Load image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not read image from '{image_path}'")
        return

    # Detection Parameters
    params = {
        "approx_epsilon_factor": 0.0035,
        "min_segment_len": 80,
        "max_segment_len": 600,
        "red_channel_thresh_g": 23,
        "morph_kernel_size_g": (6, 6),
        "red_channel_thresh_w": 22,
        "morph_kernel_size_w": (3, 3),
        "contact_distance_threshold": 40,
        "denoise_h": 5.0,
        "denoise_h_color": 3.0,
        "denoise_template_window_size": 9,
        "denoise_search_window_size": 27,
    }
    
    # Run Boundary Detection
    result = contact_boundary_detection_rgb(img, **params)
    
    if isinstance(result, tuple):
        contact_segments, _, _ = result
    else:
        contact_segments = result
    
    if not contact_segments:
        print("No contact segments found.")
        return

    print(f"Found {len(contact_segments)} segments.")
    
    # --- Categorization and Statistics ---
    descriptors = calculate_descriptors(contact_segments, PIXEL_TO_MICRON)
    
    # --- Output Results ---
    print("\n--- Segment Statistics (mm) ---")
    print(f"Total CAT_HORIZONTAL Length: {descriptors['CAT_HORIZONTAL_len_mm']:.4f}")
    print(f"Total CAT_CLOSING Length: {descriptors['CAT_CLOSING_len_mm']:.4f}")
    print(f"Total CAT_OPENING_UPPER Length: {descriptors['CAT_OPENING_UPPER_len_mm']:.4f}")
    print(f"Total CAT_OPENING_LOWER Length: {descriptors['CAT_OPENING_LOWER_len_mm']:.4f}")
    
    print("\n--- Geometric Descriptors ---")
    print(f"SAEF: {descriptors['SAEF']:.4f}")
    print(f"MID: {descriptors['MID_mm']:.4f} mm")
    print(f"VIR: {descriptors['VIR']:.4f}")

    if visualize:
        # Re-create visualization using the segments (optional: could move vis logic to function too, but keeping simple)
        colors = {
            CAT_HORIZONTAL: (255, 0, 0),       # Blue
            CAT_CLOSING: (0, 0, 255),         # Red
            CAT_OPENING_UPPER: (0, 255, 0),   # Green
            CAT_OPENING_LOWER: (0, 255, 255), # Yellow
            CAT_UNCLASSIFIED: (128, 128, 128) # Gray
        }
        
        vis_img = img.copy()
        for i, segment in enumerate(descriptors['categorized_segments']):
            p1 = segment['p1']
            p2 = segment['p2']
            category = segment['category']
            
            color = colors.get(category, (255, 255, 255))
            cv2.line(vis_img, tuple(p1), tuple(p2), color, 2)
            cv2.circle(vis_img, tuple(p1), 3, color, -1)
            
            mid_point = (p1 + p2) / 2
            mid_point_int = (int(mid_point[0]), int(mid_point[1]))
            # label = str(category)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # cv2.putText(vis_img, label, mid_point_int, font, 1.8, (0, 0, 0), 4, cv2.LINE_AA)
            # cv2.putText(vis_img, label, mid_point_int, font, 1.8, (255, 255, 255), 2, cv2.LINE_AA)

        display_scale_factor = 1.5
        base_display_width = 768
        base_display_height = 512
        vis_img_resized = cv2.resize(vis_img, (int(base_display_width * display_scale_factor), int(base_display_height * display_scale_factor)))
        cv2.imshow("Strength Aware Analysis", vis_img_resized)
        print("\nPress any key to close the visualization window...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def calculate_descriptors(contact_segments, pixel_to_micron):
    """
    Calculates geometric descriptors from contact segments.
    
    Args:
        contact_segments: List of tuples [(p1, p2), ...]
        pixel_to_micron: Fixed conversion factor.
        
    Returns:
        dict: Dictionary of descriptors and stats.
    """
    pixel_to_mm = pixel_to_micron * MICRON_TO_MM
    
    stats = {
        CAT_HORIZONTAL: 0.0,
        CAT_CLOSING: 0.0,
        CAT_OPENING_UPPER: 0.0,
        CAT_OPENING_LOWER: 0.0,
        CAT_UNCLASSIFIED: 0.0
    }
    
    categorized_segments = []
    
    for p1, p2 in contact_segments:
        category, angle = classify_segment(p1, p2)
        length_px = np.linalg.norm(p2 - p1)
        length_mm = length_px * pixel_to_mm
        
        stats[category] += length_mm
        categorized_segments.append({
            "p1": p1,
            "p2": p2,
            "category": category,
            "angle": angle,
            "length_mm": length_mm
        })
        
    # --- Descriptor Calculation ---
    
    # 1. SAEF (Surface Area Enhancement Factor)
    l_total_mm = sum(stats.values())
    
    all_points = np.array([pt for seg in contact_segments for pt in seg])
    y_coords = all_points[:, 1]
    x_coords = all_points[:, 0]
    
    l_projected_px = np.max(y_coords) - np.min(y_coords) # Seam Length in Y (pixels)
    l_projected_mm = l_projected_px * pixel_to_mm
    
    saef = l_total_mm / l_projected_mm if l_projected_mm > 0 else 0
    
    # 2. MID (Mechanical Interlocking Depth)
    mid_val_px = np.max(x_coords) - np.min(x_coords) # Amplitude in X (pixels)
    mid_val_mm = mid_val_px * pixel_to_mm
    
    # 3. VIR (Volumetric Interlock Ratio)
    # VIR is a ratio of areas, so units cancel out. We can calculate in pixels.
    vir = calculate_vir(contact_segments, mid_val_px, l_projected_px)
    
    return {
        "SAEF": saef,
        "MID_mm": mid_val_mm,
        "VIR": vir,
        "CAT_HORIZONTAL_len_mm": stats[CAT_HORIZONTAL],
        "CAT_CLOSING_len_mm": stats[CAT_CLOSING],
        "CAT_OPENING_UPPER_len_mm": stats[CAT_OPENING_UPPER],
        "CAT_OPENING_LOWER_len_mm": stats[CAT_OPENING_LOWER],
        "categorized_segments": categorized_segments
    }

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Analyze contact boundary segments.")
    parser.add_argument("image_path", help="Path to the image file.")
    parser.add_argument("--no-vis", action="store_true", help="Disable visualization.")
    args = parser.parse_args()
    
    if os.path.isfile(args.image_path):
        analyze_image(args.image_path, visualize=not args.no_vis)
    else:
        print("Invalid file path.")
