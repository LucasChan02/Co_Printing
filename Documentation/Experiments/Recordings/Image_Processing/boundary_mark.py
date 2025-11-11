import csv
import os

import cv2
import numpy as np

# from typing import Tuple


def boundary_mark(
    image_dir: str,
    output_dir: str,
    csv_path: str,
    approx_epsilon_factor: float = 0.0035,
    min_segment_len: int = 80,
    max_segment_len: int = 600,
    pixel_to_micron: float = 1.13636,
    hsv_lower_thresh: tuple[int, int, int] = (40, 40, 40),
    hsv_upper_thresh: tuple[int, int, int] = (70, 255, 255),
    morph_kernel_size: tuple[int, int] = (6, 6),
    display_images: bool = False,
    denoise_h: float = 3.0,
    denoise_h_color: float = 3.0,
    denoise_template_window_size: int = 7,
    denoise_search_window_size: int = 21,
) -> None:
    """Args:
    image_dir (str): Path to the directory containing images to process.
    output_dir (str): Path to the directory where processed images will be saved.
    csv_path (str): Path to the CSV file to save segment length data.
    approx_epsilon_factor (float): Factor for approximating contour shape.
    min_segment_len (int): Minimum length of a contour segment to be approved.
    max_segment_len (int): Maximum length of a contour segment to be approved.
    pixel_to_micron (float): Conversion factor from pixels to microns.
    hsv_lower_thresh (tuple): Lower bound for the HSV color mask.
    hsv_upper_thresh (tuple): Upper bound for the HSV color mask.
    morph_kernel_size (tuple): Kernel size for morphological closing.
    denoise_h (float): Filter strength for luminance component. 0 to disable.
    denoise_h_color (float): Filter strength for color components.
    denoise_template_window_size (int): Template patch size for denoising.
    denoise_search_window_size (int): Search window size for denoising.
    """
    # Create output directory if it doesn't exist
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Open CSV file for writing
    with open(csv_path, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(
            ["image_filename", "total_length_microns", "total_length_mm"]
        )

        # List all files in the image directory
        for filename in os.listdir(image_dir):
            # Check for common image file extensions
            if filename.lower().endswith((".png", ".jpg", ".jpeg", ".bmp", ".tiff")):
                image_path = os.path.join(image_dir, filename)

                # 1. Load Image
                img = cv2.imread(image_path)
                if img is None:
                    print(f"Warning: Could not read image {filename}. Skipping.")
                    continue

                # Denoise the image if strength is specified
                if denoise_h > 0:
                    img = cv2.fastNlMeansDenoisingColored(
                        img,
                        None,
                        denoise_h,
                        denoise_h_color,
                        denoise_template_window_size,
                        denoise_search_window_size,
                    )

                # Create output images
                output_img_approx = img.copy()
                processed_lines_img = np.zeros_like(img)

                # 2. Convert to HSV and create mask
                hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
                lower_green = np.array(hsv_lower_thresh)
                upper_green = np.array(hsv_upper_thresh)
                mask_green = cv2.inRange(hsv, lower_green, upper_green)

                # 3. Clean the mask
                kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_size)
                mask_green = cv2.morphologyEx(mask_green, cv2.MORPH_CLOSE, kernel)

                # 4. Find green contours
                contours, Mat = cv2.findContours(
                    mask_green, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
                )

                # 5. Process each contour
                total_length_pixels = 0
                for cnt in contours:
                    epsilon = approx_epsilon_factor * cv2.arcLength(cnt, True)
                    approx = cv2.approxPolyDP(cnt, epsilon, True)
                    Mat = cv2.polylines(
                        output_img_approx,
                        [approx],
                        isClosed=True,
                        color=(255, 0, 0),
                        thickness=2,
                    )

                    for i in range(len(approx)):
                        p1 = approx[i][0]
                        p2 = approx[(i + 1) % len(approx)][0]
                        segment_length = np.linalg.norm(p1 - p2)

                        if min_segment_len <= segment_length <= max_segment_len:
                            total_length_pixels += segment_length
                            _ = cv2.line(
                                processed_lines_img,
                                tuple(p1),
                                tuple(p2),
                                (0, 255, 0),
                                2,
                            )

                        """
                        else:  #  Do not draw
                            Mat = cv2.line(
                                processed_lines_img,
                                tuple(p1),
                                tuple(p2),
                                (0, 0, 255),
                                2,
                            )
                        """

                # 6. Convert length and save results
                length_microns = total_length_pixels * pixel_to_micron
                length_mm = length_microns / 1000
                print(
                    f"Processed {filename}: Total length of approved segments ≈ {length_mm:.4f} mm"
                )

                # Write to CSV
                csv_writer.writerow([filename, length_microns, length_mm])

                # 7. Save the processed images
                approx_output_path = os.path.join(
                    output_dir, f"{os.path.splitext(filename)[0]}_approximated.png"
                )
                segments_output_path = os.path.join(
                    output_dir, f"{os.path.splitext(filename)[0]}_segments.png"
                )
                resized_approx = cv2.resize(output_img_approx, (768, 512))
                _ = cv2.imwrite(approx_output_path, resized_approx)
                _ = cv2.imwrite(segments_output_path, processed_lines_img)

                # 8. Display images if flag is set
                if display_images:
                    cv2.namedWindow("Approximated", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("Approximated", 768, 512)
                    cv2.imshow("Approximated", output_img_approx)
                    cv2.namedWindow("Segments", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("Segments", 768, 512)
                    cv2.imshow("Segments", processed_lines_img)
                    cv2.waitKey(0)
                    cv2.destroyAllWindows()

    print("-" * 30)
    print(f"Processing complete. Processed images saved to '{output_dir}'.")
    print(f"Segment length data saved to '{csv_path}'.")
