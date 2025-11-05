from boundary_mark import boundary_mark
import os

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
        # approx_epsilon_factor=0.0035,
        # min_segment_len=60,
        # max_segment_len=600,
        # pixel_to_micron=1.13636,
        # hsv_lower_thresh=(40, 40, 40),
        # hsv_upper_thresh=(70, 255, 255),
        # morph_kernel_size=(5, 5),
    )
