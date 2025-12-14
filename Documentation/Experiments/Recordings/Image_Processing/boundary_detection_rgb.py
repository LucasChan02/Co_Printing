from typing import List, Tuple, Union

import cv2
import numpy as np


def contact_boundary_detection_rgb(
    img: np.ndarray,
    approx_epsilon_factor: float = 0.0035,
    min_segment_len: int = 80,
    max_segment_len: int = 600,
    red_channel_thresh_g: int = 50,
    morph_kernel_size_g: tuple[int, int] = (6, 6),
    red_channel_thresh_w: int = 200,
    morph_kernel_size_w: tuple[int, int] = (6, 6),
    contact_distance_threshold: int = 10,
    denoise_h: float = 3.0,
    denoise_h_color: float = 3.0,
    denoise_template_window_size: int = 7,
    denoise_search_window_size: int = 21,
    visualize: bool = True,
) -> Union[List[tuple], Tuple[List[tuple], np.ndarray, np.ndarray]]:
    """
    Detects the contact boundary between two materials in a single image.

    Args:
        img (np.ndarray): The input image as a NumPy array.
        ... (other parameters) ...
        visualize (bool): If True, generates and returns visualization images.

    Returns:
        If visualize is False:
        - List[tuple]: A list of contact segments, sorted by y-coordinate.
        If visualize is True:
        - Tuple containing:
            - List[tuple]: The list of contact segments.
            - np.ndarray: An image with all detected contours drawn.
            - np.ndarray: An image showing only the final contact line segments.
    """
    if denoise_h > 0:
        processed_img = cv2.fastNlMeansDenoisingColored(
            img,
            None,
            denoise_h,
            denoise_h_color,
            denoise_template_window_size,
            denoise_search_window_size,
        )
    else:
        processed_img = img.copy()

    # --- RGB Channel Splitting and Masking ---
    _, _, r_channel = cv2.split(processed_img)

    # --- Green Material Processing ---
    _, mask_g = cv2.threshold(
        r_channel, red_channel_thresh_g, 255, cv2.THRESH_BINARY_INV
    )
    kernel_g = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_size_g)
    mask_g = cv2.morphologyEx(mask_g, cv2.MORPH_CLOSE, kernel_g)
    contours_g, _ = cv2.findContours(mask_g, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # --- White Material Processing ---
    _, mask_w = cv2.threshold(r_channel, red_channel_thresh_w, 255, cv2.THRESH_BINARY)
    kernel_w = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, morph_kernel_size_w)
    mask_w = cv2.morphologyEx(mask_w, cv2.MORPH_CLOSE, kernel_w)
    contours_w, _ = cv2.findContours(mask_w, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # --- Approximate Contours and Store Segments ---
    segments_g = []
    for cnt in contours_g:
        epsilon = approx_epsilon_factor * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % len(approx)][0]
            segments_g.append((p1, p2))

    segments_w = []
    for cnt in contours_w:
        epsilon = approx_epsilon_factor * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, epsilon, True)
        for i in range(len(approx)):
            p1 = approx[i][0]
            p2 = approx[(i + 1) % len(approx)][0]
            segments_w.append((p1, p2))

    # --- Identify and Store Contact Segments ---
    contact_segments = []
    if segments_g and segments_w:
        for seg_g in segments_g:
            p1_g, p2_g = seg_g
            len_g = np.linalg.norm(p1_g - p2_g)

            if not (min_segment_len <= len_g <= max_segment_len):
                continue

            mid_g = (p1_g + p2_g) / 2
            min_dist = float("inf")
            best_seg_w = None

            for seg_w in segments_w:
                mid_w = (seg_w[0] + seg_w[1]) / 2
                dist = np.linalg.norm(mid_g - mid_w)
                if dist < min_dist:
                    min_dist = dist
                    best_seg_w = seg_w

            if min_dist < contact_distance_threshold and best_seg_w is not None:
                # Interpolate between the Green segment and the closest White segment
                p1_w, p2_w = best_seg_w
                
                # 1. Align directions
                vec_g = p2_g - p1_g
                vec_w = p2_w - p1_w
                
                # Check dot product to see if segments are antiparallel
                dot_prod = np.dot(vec_g, vec_w)
                
                if dot_prod < 0:
                    # Flip white segment points for interpolation if they run in opposite directions
                    p1_w_aligned = p2_w
                    p2_w_aligned = p1_w
                else:
                    p1_w_aligned = p1_w
                    p2_w_aligned = p2_w
                
                # 2. Average the coordinates
                p1_avg = (p1_g.astype(np.float32) + p1_w_aligned.astype(np.float32)) / 2.0
                p2_avg = (p2_g.astype(np.float32) + p2_w_aligned.astype(np.float32)) / 2.0
                
                # Convert back to int for cv2 drawing compatibility (optional, but good for pixels)
                p1_avg_int = p1_avg.astype(np.int32)
                p2_avg_int = p2_avg.astype(np.int32)

                contact_segments.append((p1_avg_int, p2_avg_int))

    # Sort segments from top to bottom
    contact_segments.sort(key=lambda seg: (seg[0][1] + seg[1][1]) / 2)

    if visualize:
        output_img_contours = img.copy()
        contact_lines_img = np.zeros_like(img)

        # Draw all contours
        all_contours_g = [
            cv2.approxPolyDP(c, 0.0035 * cv2.arcLength(c, True), True)
            for c in contours_g
        ]
        cv2.polylines(
            output_img_contours,
            all_contours_g,
            isClosed=True,
            color=(255, 0, 0),
            thickness=2,
        )

        all_contours_w = [
            cv2.approxPolyDP(c, 0.0035 * cv2.arcLength(c, True), True)
            for c in contours_w
        ]
        cv2.polylines(
            output_img_contours,
            all_contours_w,
            isClosed=True,
            color=(0, 0, 255),
            thickness=2,
        )

        # Draw the final sorted contact segments
        for p1, p2 in contact_segments:
            cv2.line(contact_lines_img, tuple(p1), tuple(p2), (0, 255, 0), 2)

        return contact_segments, output_img_contours, contact_lines_img
    else:
        return contact_segments
