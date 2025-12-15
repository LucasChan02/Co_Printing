import os

import cv2

from boundary_detection_rgb import contact_boundary_detection_rgb
from image_output import save_csv_data, save_visualization_images

from descriptor_analysis import calculate_descriptors

if __name__ == "__main__":
    CWD = os.path.dirname(os.path.realpath(__file__))
    IMAGE_DIR = os.path.join(CWD, "..", "Micro/cap_1113")
    OUTPUT_DIR = os.path.join(IMAGE_DIR, "processed_split_1113")

    # --- Processing Parameters ---
    PIXEL_TO_MICRON = 1.13636
    PARAMS = {
        "approx_epsilon_factor": 0.0035,
        "min_segment_len": 80,
        "max_segment_len": 900,
        "red_channel_thresh_g": 65,
        "morph_kernel_size_g": (15, 15),
        "red_channel_thresh_w": 80,
        "morph_kernel_size_w": (15, 15),
        "contact_distance_threshold": 140,
        "denoise_h": 15.0,
        "denoise_h_color": 10.0,
        "denoise_template_window_size": 9,
        "denoise_search_window_size": 27,
    }
    # --- End of Parameters ---

    if not os.path.exists(OUTPUT_DIR):
        os.makedirs(OUTPUT_DIR)

    print(f"Image Source: {IMAGE_DIR}")
    print(f"Outputting to: {OUTPUT_DIR}")
    print("-" * 30)

    all_image_data = {}

    for filename in os.listdir(IMAGE_DIR):
        if not filename.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tiff")):
            continue

        image_path = os.path.join(IMAGE_DIR, filename)
        img = cv2.imread(image_path)
        if img is None:
            print(f"Warning: Could not read image {filename}. Skipping.")
            continue

        print(f"Processing {filename}...")

        # Call the refactored function to get segments and visualization images
        result = contact_boundary_detection_rgb(img=img, visualize=True, **PARAMS)
        contact_segments, contours_img, lines_img = result

        # Calculate Geometric Descriptors
        descriptors = calculate_descriptors(contact_segments, PIXEL_TO_MICRON)
        
        # Store both segments and descriptors
        all_image_data[filename] = {
            "segments": contact_segments,
            "descriptors": descriptors
        }

        # Save the visualization images using the new output function
        save_visualization_images(OUTPUT_DIR, filename, contours_img, lines_img)

    # After processing all images, save the data to CSV files
    save_csv_data(OUTPUT_DIR, all_image_data, PIXEL_TO_MICRON)

    print("-" * 30)
    print("Processing complete.")
