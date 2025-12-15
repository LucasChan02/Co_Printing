import argparse
import os

import cv2

# import numpy as np
from boundary_detection_rgb import contact_boundary_detection_rgb
from image_output import display_images

if __name__ == "__main__":
    CWD = os.path.dirname(os.path.realpath(__file__))
    IMAGE_DIR = os.path.join(CWD, "..", "Micro/cap_1113")
    FILENAME = "slo_40_01_0002.png"  # TODO: Update this filename to the image you want to test
    
    image_path = os.path.join(IMAGE_DIR, FILENAME)

    if not os.path.isfile(image_path):
        print(f"Error: File not found at '{image_path}'")
        exit(1)

    # --- Processing Parameters for Testing ---
    # Adjust these values to tune the detection
    PARAMS = {
        "approx_epsilon_factor": 0.0035,
        "min_segment_len": 80,
        "max_segment_len": 600,
        "red_channel_thresh_g": 65,
        "morph_kernel_size_g": (15, 15),
        "red_channel_thresh_w": 80,
        "morph_kernel_size_w": (15, 15),
        "contact_distance_threshold": 100,
        "denoise_h": 15.0,
        "denoise_h_color": 10.0,
        "denoise_template_window_size": 9,
        "denoise_search_window_size": 27,
    }
    # --- End of Parameters ---

    # Load the image
    img = cv2.imread(image_path)
    if img is None:
        print(f"Error: Could not read image from '{image_path}'")
        exit(1)

    print("--- Single Image Test ---")
    print(f"Processing: {image_path}")
    print("-" * 30)

    # Run the processing function on the single image with visualization enabled
    contact_segments, contours_img, lines_img = contact_boundary_detection_rgb(
        img=img, visualize=True, **PARAMS
    )

    print(f"Found {len(contact_segments)} contact segments.")
    for i, (p1, p2) in enumerate(contact_segments):
        print(f"  Segment {i}: Start({p1[0]}, {p1[1]}), End({p2[0]}, {p2[1]})")

    print("-" * 30)
    
    # Calculate Geometric Descriptors
    PIXEL_TO_MICRON = 1.13636
    from descriptor_analysis import calculate_descriptors
    descriptors = calculate_descriptors(contact_segments, PIXEL_TO_MICRON)
    
    print("\n--- Segment Statistics (mm) ---")
    print(f"Total CAT_HORIZONTAL Length: {descriptors['CAT_HORIZONTAL_len_mm']:.4f}")
    print(f"Total CAT_CLOSING Length: {descriptors['CAT_CLOSING_len_mm']:.4f}")
    print(f"Total CAT_OPENING_UPPER Length: {descriptors['CAT_OPENING_UPPER_len_mm']:.4f}")
    print(f"Total CAT_OPENING_LOWER Length: {descriptors['CAT_OPENING_LOWER_len_mm']:.4f}")
    
    print("\n--- Geometric Descriptors ---")
    print(f"SAEF: {descriptors['SAEF']:.4f}")
    print(f"MID: {descriptors['MID_mm']:.4f} mm")
    print(f"VIR: {descriptors['VIR']:.4f}")
    print("-" * 30)

    # Display images
    display_images(contours_img, lines_img)

    print("Test complete.")
