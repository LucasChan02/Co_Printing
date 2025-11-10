import csv
import os

import cv2
import numpy as np


def contact_boundary_detection_rgb(
    image_dir: str,
    output_dir: str,
    csv_path: str,
    approx_epsilon_factor: float = 0.0035,
    min_segment_len: int = 80,
    max_segment_len: int = 600,
    pixel_to_micron: float = 1.13636,
    red_channel_thresh_g: int = 50,
    morph_kernel_size_g: tuple[int, int] = (6, 6),
    red_channel_thresh_w: int = 200,
    morph_kernel_size_w: tuple[int, int] = (6, 6),
    contact_distance_threshold: int = 10,
    display_images: bool = False,
    denoise_h: float = 3.0,
    denoise_h_color: float = 3.0,
    denoise_template_window_size: int = 7,
    denoise_search_window_size: int = 21,
) -> None:
    """
    Detects and measures the contact boundary between two materials using RGB channel analysis.

    Args:
        image_dir (str): Path to the directory containing images.
        output_dir (str): Path to save processed images.
        csv_path (str): Path to save segment length data.
        approx_epsilon_factor (float): Factor for approximating contour shape.
        min_segment_len (int): Minimum length of a contour segment to be approved.
        max_segment_len (int): Maximum length of a contour segment to be approved.
        pixel_to_micron (float): Conversion factor from pixels to microns.
        red_channel_thresh_g (int): Upper threshold for the red channel for green material.
        morph_kernel_size_g (tuple): Morphological kernel size for the green mask.
        red_channel_thresh_w (int): Lower threshold for the red channel for white material.
        morph_kernel_size_w (tuple): Morphological kernel size for the white mask.
        contact_distance_threshold (int): Max distance (pixels) between midpoints
                                           of segments to be considered in contact.
        display_images (bool): Whether to display images during processing.
        denoise_h (float): Denoising filter strength for luminance.
        denoise_h_color (float): Denoising filter strength for color.
        denoise_template_window_size (int): Denoising template window size.
        denoise_search_window_size (int): Denoising search window size.
    """
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    with open(csv_path, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(
            ["image_filename", "contact_length_microns", "contact_length_mm"]
        )

        for filename in os.listdir(image_dir):
            if not filename.lower().endswith(
                (".png", ".jpg", ".jpeg", ".bmp", ".tiff")
            ):
                continue

            image_path = os.path.join(image_dir, filename)
            img = cv2.imread(image_path)
            if img is None:
                print(f"Warning: Could not read image {filename}. Skipping.")
                continue

            if denoise_h > 0:
                img = cv2.fastNlMeansDenoisingColored(
                    img,
                    None,
                    denoise_h,
                    denoise_h_color,
                    denoise_template_window_size,
                    denoise_search_window_size,
                )

            output_img_contours = img.copy()
            contact_lines_img = np.zeros_like(img)

            # --- RGB Channel Splitting and Masking ---
            _, _, r_channel = cv2.split(img)

            # --- Green Material Processing (low red channel values) ---
            _, mask_g = cv2.threshold(
                r_channel, red_channel_thresh_g, 255, cv2.THRESH_BINARY_INV
            )
            kernel_g = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_size_g)
            mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_CLOSE, kernel_g)
            contours_g, _ = cv2.findContours(
                mask_g, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # --- White Material Processing (high red channel values) ---
            _, mask_w = cv2.threshold(
                r_channel, red_channel_thresh_w, 255, cv2.THRESH_BINARY
            )
            kernel_w = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_size_w)
            mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_CLOSE, kernel_w)
            contours_w, _ = cv2.findContours(
                mask_w, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            # --- Approximate Contours and Store Segments ---
            segments_g = []
            for cnt in contours_g:
                epsilon = approx_epsilon_factor * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                cv2.polylines(
                    output_img_contours,
                    [approx],
                    isClosed=True,
                    color=(255, 0, 0),
                    thickness=2,
                )
                for i in range(len(approx)):
                    p1 = approx[i][0]
                    p2 = approx[(i + 1) % len(approx)][0]
                    segments_g.append((p1, p2))

            segments_w = []
            for cnt in contours_w:
                epsilon = approx_epsilon_factor * cv2.arcLength(cnt, True)
                approx = cv2.approxPolyDP(cnt, epsilon, True)
                cv2.polylines(
                    output_img_contours,
                    [approx],
                    isClosed=True,
                    color=(0, 0, 255),
                    thickness=2,
                )
                for i in range(len(approx)):
                    p1 = approx[i][0]
                    p2 = approx[(i + 1) % len(approx)][0]
                    segments_w.append((p1, p2))

            # --- Identify Contact Segments ---
            total_contact_length_pixels = 0
            if not segments_g or not segments_w:
                print(
                    f"Warning: No segments found for one or both materials in {filename}. Skipping contact analysis."
                )
            else:
                for seg_g in segments_g:
                    p1_g, p2_g = seg_g
                    mid_g = (p1_g + p2_g) / 2
                    len_g = np.linalg.norm(p1_g - p2_g)

                    if not (min_segment_len <= len_g <= max_segment_len):
                        continue

                    min_dist = float("inf")
                    for seg_w in segments_w:
                        p1_w, p2_w = seg_w
                        mid_w = (p1_w + p2_w) / 2
                        dist = np.linalg.norm(mid_g - mid_w)
                        if dist < min_dist:
                            min_dist = dist

                    if min_dist < contact_distance_threshold:
                        total_contact_length_pixels += len_g
                        cv2.line(
                            contact_lines_img, tuple(p1_g), tuple(p2_g), (0, 255, 0), 2
                        )

            # --- Save Results ---
            length_microns = total_contact_length_pixels * pixel_to_micron
            length_mm = length_microns / 1000
            print(
                f"Processed {filename}: Total contact length (RGB) ≈ {length_mm:.4f} mm"
            )

            csv_writer.writerow([filename, length_microns, length_mm])

            contour_output_path = os.path.join(
                output_dir, f"{os.path.splitext(filename)[0]}_contours_rgb.png"
            )
            segments_output_path = os.path.join(
                output_dir, f"{os.path.splitext(filename)[0]}_contact_segments_rgb.png"
            )

            resized_contours = cv2.resize(output_img_contours, (768, 512))
            cv2.imwrite(contour_output_path, resized_contours)
            cv2.imwrite(segments_output_path, contact_lines_img)

            if display_images:
                cv2.namedWindow("All Contours (RGB)", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("All Contours (RGB)", 768, 512)
                cv2.imshow("All Contours (RGB)", output_img_contours)

                cv2.namedWindow("Contact Segments (RGB)", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("Contact Segments (RGB)", 768, 512)
                cv2.imshow("Contact Segments (RGB)", contact_lines_img)

                cv2.waitKey(0)
                cv2.destroyAllWindows()

    print("-" * 30)
    print(f"Processing complete. Processed images saved to '{output_dir}'.")
    print(f"Contact length data saved to '{csv_path}'.")
