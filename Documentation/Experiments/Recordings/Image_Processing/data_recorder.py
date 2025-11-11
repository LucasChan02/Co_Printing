import csv

import numpy as np


def save_contact_data(
    csv_path: str,
    all_image_data: dict,
    pixel_to_micron: float,
):
    """
    Saves all collected contact segment data to a detailed CSV file.

    Args:
        csv_path (str): Path to the output CSV file.
        all_image_data (dict): A dictionary where keys are image filenames and
                               values are lists of contact segments.
        pixel_to_micron (float): Conversion factor from pixels to microns.
    """
    with open(csv_path, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(
            [
                "image_filename",
                "segment_index",
                "start_x",
                "start_y",
                "end_x",
                "end_y",
                "length_pixels",
                "length_microns",
            ]
        )

        for filename, segments in all_image_data.items():
            if not segments:
                csv_writer.writerow([filename, "N/A", "N/A", "N/A", "N/A", "N/A", 0, 0])
            else:
                for i, segment in enumerate(segments):
                    p1, p2 = segment
                    length_pixels = np.linalg.norm(p1 - p2)
                    length_microns = length_pixels * pixel_to_micron
                    csv_writer.writerow(
                        [
                            filename,
                            i,
                            p1[0],
                            p1[1],
                            p2[0],
                            p2[1],
                            f"{length_pixels:.4f}",
                            f"{length_microns:.4f}",
                        ]
                    )


def save_summary_data(
    summary_csv_path: str, all_image_data: dict, pixel_to_micron: float
):
    """
    Saves a summary of contact lengths for each image to a CSV file.

    Args:
        summary_csv_path (str): Path to the output summary CSV file.
        all_image_data (dict): A dictionary where keys are image filenames and
                               values are lists of contact segments.
        pixel_to_micron (float): Conversion factor from pixels to microns.
    """
    with open(summary_csv_path, "w", newline="") as csvfile:
        csv_writer = csv.writer(csvfile)
        csv_writer.writerow(
            [
                "image_filename",
                "total_contact_length_microns",
                "total_contact_length_mm",
            ]
        )
        for filename, segments in all_image_data.items():
            total_contact_length_pixels = sum(
                np.linalg.norm(p1 - p2) for p1, p2 in segments
            )
            length_microns = total_contact_length_pixels * pixel_to_micron
            length_mm = length_microns / 1000
            csv_writer.writerow([filename, f"{length_microns:.4f}", f"{length_mm:.4f}"])
