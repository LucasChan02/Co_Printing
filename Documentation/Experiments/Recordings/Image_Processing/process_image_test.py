import argparse
import os

import cv2

# import numpy as np
from boundary_detection_rgb import contact_boundary_detection_rgb
from image_output import display_images

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run contact_boundary_detection_rgb on a single image for testing with display enabled."
    )
    parser.add_argument("image_path", help="Path to the image file.")
    args = parser.parse_args()

    if not os.path.isfile(args.image_path):
        print(f"Error: File not found at '{args.image_path}'")
        exit(1)

    # --- Processing Parameters for Testing ---
    # Adjust these values to tune the detection
    PARAMS = {
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
    # --- End of Parameters ---

    # Load the image
    img = cv2.imread(args.image_path)
    if img is None:
        print(f"Error: Could not read image from '{args.image_path}'")
        exit(1)

    print("--- Single Image Test ---")
    print(f"Processing: {args.image_path}")
    print("-" * 30)

    # Run the processing function on the single image with visualization enabled
    contact_segments, contours_img, lines_img = contact_boundary_detection_rgb(
        img=img, visualize=True, **PARAMS
    )

    print(f"Found {len(contact_segments)} contact segments.")
    for i, (p1, p2) in enumerate(contact_segments):
        print(f"  Segment {i}: Start({p1[0]}, {p1[1]}), End({p2[0]}, {p2[1]})")

    print("-" * 30)

    # Display images
    display_images(contours_img, lines_img)

    print("Test complete.")
