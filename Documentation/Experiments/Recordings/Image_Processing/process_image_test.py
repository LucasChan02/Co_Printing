import argparse
import os
import shutil
import tempfile

from boundary_mark import boundary_mark

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Run boundary_mark on a single image for testing."
    )
    parser.add_argument(
        "image_path",
        help="Path to image",
    )
    # parser.add_argument(
    #     "--denoise-h",
    #     type=float,
    #     default=10.0,
    #     help="Denoising filter strength for luminance. Default is 10.0, 0 to disable.",
    # )
    # parser.add_argument(
    #     "--denoise-h-color",
    #     type=float,
    #     default=10.0,
    #     help="Denoising filter strength for color. Default is 10.0.",
    # )
    # parser.add_argument(
    #     "--denoise-template-window-size",
    #     type=int,
    #     default=7,
    #     help="Denoising template window size. Should be odd. Default is 7.",
    # )
    # parser.add_argument(
    #     "--denoise-search-window-size",
    #     type=int,
    #     default=21,
    #     help="Denoising search window size. Default is 21.",
    # )
    args = parser.parse_args()

    image_path = args.image_path
    if not os.path.isfile(image_path):
        print(f"Error: File not found at '{image_path}'")
        exit(1)

    # Create a temporary directory to isolate the single image for processing
    with tempfile.TemporaryDirectory() as temp_dir:
        # Copy the image to the temporary directory
        shutil.copy(image_path, temp_dir)

        # Define output paths within the temporary directory
        output_dir = os.path.join(temp_dir, "processed")
        csv_path = os.path.join(output_dir, "segment_lengths.csv")

        print("--- Single Image Test ---")
        print(f"Processing: {image_path}")
        print(f"Using temporary directory: {temp_dir}")
        print("Displaying results visually.")
        print("-" * 30)

        # Run the processing function
        boundary_mark(
            image_dir=temp_dir,
            output_dir=output_dir,
            csv_path=csv_path,
            min_segment_len=70,
            max_segment_len=500,
            pixel_to_micron=0.844451,
            hsv_lower_thresh=(60, 140, 18),
            hsv_upper_thresh=(130, 255, 255),
            morph_kernel_size=(7, 7),
            display_images=True,
            denoise_h=6,
            denoise_h_color=8,
            denoise_template_window_size=7,
            denoise_search_window_size=21,
        )

        print("-" * 30)
