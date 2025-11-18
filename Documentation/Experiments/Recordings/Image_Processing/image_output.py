import os

import cv2

from data_recorder import save_contact_data, save_summary_data


def save_visualization_images(
    output_dir, filename, contours_img, lines_img, resize_dim=(768, 512)
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

    # The lines_img is saved without resizing, as in the original script
    cv2.imwrite(segments_output_path, lines_img)


def display_images(contours_img, lines_img, window_size=(768, 512)):
    """
    Displays contour and segment images in resizable windows until a key is pressed.

    Args:
        contours_img (numpy.ndarray): The image showing all detected contours.
        lines_img (numpy.ndarray): The image showing the final contact segments.
        window_size (tuple, optional): The initial size (width, height) for the
                                       display windows. Defaults to (768, 512).
    """
    cv2.namedWindow("All Contours", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("All Contours", window_size[0], window_size[1])
    cv2.imshow("All Contours", contours_img)

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

    # Define paths for the new CSV output files
    detailed_csv_path = os.path.join(output_dir, "contact_segments_detailed.csv")
    summary_csv_path = os.path.join(output_dir, "contact_lengths_summary.csv")

    print(f"Saving detailed segment data to {detailed_csv_path}")
    save_contact_data(detailed_csv_path, all_image_data, pixel_to_micron)

    print(f"Saving summary data to {summary_csv_path}")
    save_summary_data(summary_csv_path, all_image_data, pixel_to_micron)
