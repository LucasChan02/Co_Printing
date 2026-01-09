import os

import cv2

from data_recorder import save_contact_data, save_summary_data


import numpy as np

def draw_trend_line_on_image(img, trend_line):
    """Draws the trend line as a yellow dashed line on the image."""
    if not trend_line:
        return

    for ((x1, y1), (x2, y2)) in trend_line:
        pt1 = (int(x1), int(y1))
        pt2 = (int(x2), int(y2))
        
        dash_len = 10
        gap_len = 10
        total_dist = np.hypot(pt2[0]-pt1[0], pt2[1]-pt1[1])
        if total_dist == 0:
            continue
            
        vx = (pt2[0]-pt1[0]) / total_dist
        vy = (pt2[1]-pt1[1]) / total_dist
        
        curr_dist = 0
        while curr_dist < total_dist:
            p_start = (int(pt1[0] + vx * curr_dist), int(pt1[1] + vy * curr_dist))
            p_end_dist = min(curr_dist + dash_len, total_dist)
            p_end = (int(pt1[0] + vx * p_end_dist), int(pt1[1] + vy * p_end_dist))
            
            cv2.line(img, p_start, p_end, (0, 255, 255), 2)
            curr_dist += dash_len + gap_len

def save_visualization_images(
    output_dir, filename, contours_img, lines_img, resize_dim=(768, 512), 
    trend_line=None, draw_trend_line=False
):
    """
    Saves contour and segment visualization images to the specified directory.

    Args:
        output_dir (str): The directory to save the images.
        filename (str): The original filename, used to create output names.
        contours_img (numpy.ndarray): The image showing all detected contours.
        lines_img (numpy.ndarray): The image showing the final contact segments.
        resize_dim (tuple, optional): The dimension (width, height) to resize the
                                      contour image. Defaults to (768, 512).
                                      If None, no resizing is done.
        trend_line (list, optional): List of trend line segments.
        draw_trend_line (bool, optional): Whether to draw the trend line.
    """
    contour_output_path = os.path.join(
        output_dir, f"{os.path.splitext(filename)[0]}_contours.png"
    )
    segments_output_path = os.path.join(
        output_dir, f"{os.path.splitext(filename)[0]}_contact_segments.png"
    )

    # Resize for consistent output size if desired
    if resize_dim:
        resized_contours = cv2.resize(contours_img, resize_dim)
        cv2.imwrite(contour_output_path, resized_contours)
    else:
        cv2.imwrite(contour_output_path, contours_img)

    # Draw trend line if requested
    if draw_trend_line and trend_line:
        draw_trend_line_on_image(lines_img, trend_line)

    # The lines_img is saved without resizing, as in the original script
    cv2.imwrite(segments_output_path, lines_img)


def display_images(contours_img, lines_img, window_size=(768, 512), 
                   trend_line=None, draw_trend_line=False):
    """
    Displays contour and segment images in resizable windows until a key is pressed.

    Args:
        contours_img (numpy.ndarray): The image showing all detected contours.
        lines_img (numpy.ndarray): The image showing the final contact segments.
        window_size (tuple, optional): The initial size (width, height) for the
                                       display windows. Defaults to (768, 512).
        trend_line (list, optional): List of trend line segments.
        draw_trend_line (bool, optional): Whether to draw the trend line.
    """
    cv2.namedWindow("All Contours", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("All Contours", window_size[0], window_size[1])
    cv2.imshow("All Contours", contours_img)

    # Draw trend line if requested
    if draw_trend_line and trend_line:
        draw_trend_line_on_image(lines_img, trend_line)

    cv2.namedWindow("Contact Segments", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Contact Segments", window_size[0], window_size[1])
    cv2.imshow("Contact Segments", lines_img)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def save_csv_data(output_dir, all_image_data, pixel_to_micron):
    """
    Saves detailed and summary data to CSV files in the specified directory.

    Args:
        output_dir (str): The directory to save the CSV files.
        all_image_data (dict): A dictionary containing segment data for all images.
        pixel_to_micron (float): The conversion factor from pixels to microns.
    """
    if not all_image_data:
        print("No data to save.")
        return

    first_filename = next(iter(all_image_data.keys()))
    specimen_name = os.path.splitext(first_filename)[0].rsplit('_', 1)[0]
    
    # Define paths for the new CSV output files
    detailed_csv_path = os.path.join(output_dir, f"{specimen_name}_contact_segments.csv")
    summary_csv_path = os.path.join(output_dir, f"{specimen_name}_contact_summary.csv")

    print(f"Saving detailed segment data to {detailed_csv_path}")
    save_contact_data(detailed_csv_path, all_image_data, pixel_to_micron)

    print(f"Saving summary data to {summary_csv_path}")
    save_summary_data(summary_csv_path, all_image_data, pixel_to_micron)
