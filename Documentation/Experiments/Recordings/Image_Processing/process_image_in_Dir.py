import os

import cv2

from boundary_detection_rgb import contact_boundary_detection_rgb
from image_output import save_csv_data, save_visualization_images

if __name__ == "__main__":
    CWD = os.path.dirname(os.path.realpath(__file__))
    IMAGE_DIR = os.path.join(CWD, "..", "Micro/4x")
    OUTPUT_DIR = os.path.join(IMAGE_DIR, "processed_images_split")

    # --- Processing Parameters ---
    PIXEL_TO_MICRON = 1.13636
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

        all_image_data[filename] = contact_segments

        # Save the visualization images using the new output function
        save_visualization_images(OUTPUT_DIR, filename, contours_img, lines_img)

    # After processing all images, save the data to CSV files
    save_csv_data(OUTPUT_DIR, all_image_data, PIXEL_TO_MICRON)

    print("-" * 30)
    print("Processing complete.")
