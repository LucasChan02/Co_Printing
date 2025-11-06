import os

from boundary_mark import boundary_mark

if __name__ == "__main__":
    CWD = os.path.dirname(os.path.realpath(__file__))
    IMAGE_DIR = os.path.join(CWD, "..", "Micro/4x")
    OUTPUT_DIR = os.path.join(IMAGE_DIR, "processed_images")
    CSV_PATH = os.path.join(IMAGE_DIR, "segment_lengths.csv")

    print(f"Image Source: {IMAGE_DIR}")
    print(f"Outputting to: {OUTPUT_DIR}")
    print("-" * 30)

    boundary_mark(
        image_dir=IMAGE_DIR,
        output_dir=OUTPUT_DIR,
        csv_path=CSV_PATH,
        min_segment_len=85,
        max_segment_len=500,
        pixel_to_micron=0.844451,
        hsv_lower_thresh=(60, 140, 18),
        hsv_upper_thresh=(130, 255, 255),
        morph_kernel_size=(5, 5),
        display_images=False,
        denoise_h=6,
        denoise_h_color=8,
        denoise_template_window_size=7,
        denoise_search_window_size=21,
    )
