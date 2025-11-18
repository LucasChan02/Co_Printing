import os
from glob import glob

import cv2
import numpy as np


def load_images_sorted(path):
    """Loads and sorts images from a folder by filename."""
    files = []
    for ext in ["*.png", "*.jpg", "*.jpeg", "*.tif", "*.tiff"]:
        files.extend(glob(os.path.join(path, ext)))

    if not files:
        raise ValueError(f"No images found in {path}")

    files = sorted(files)
    return [cv2.imread(f, cv2.IMREAD_COLOR) for f in files]


def _calculate_shifts(images):
    """Calculates translation shifts between adjacent images using phase correlation."""
    shifts = []
    for i in range(len(images) - 1):
        img1_gray = cv2.cvtColor(images[i], cv2.COLOR_BGR2GRAY)
        img2_gray = cv2.cvtColor(images[i + 1], cv2.COLOR_BGR2GRAY)
        img1_float = img1_gray.astype(np.float32)
        img2_float = img2_gray.astype(np.float32)
        shift, _ = cv2.phaseCorrelate(img1_float, img2_float)
        shifts.append(shift)
    return shifts


def stitch_row(images):
    """
    Stitches a list of images in a row using 2D translation.
    Optimized for microscopy by preserving detail and accuracy.
    """
    if not images:
        return None
    if len(images) == 1:
        return images[0]

    relative_shifts = _calculate_shifts(images)

    global_offsets = [(0, 0)]
    accumulated_offset = np.array([0.0, 0.0])
    for shift in relative_shifts:
        accumulated_offset += shift
        global_offsets.append(tuple(accumulated_offset))

    h_base, w_base, _ = images[0].shape
    min_x, max_x = 0.0, float(w_base)
    min_y, max_y = 0.0, float(h_base)

    for i in range(1, len(images)):
        h, w, _ = images[i].shape
        dx, dy = global_offsets[i]
        min_x = min(min_x, dx)
        max_x = max(max_x, dx + w)
        min_y = min(min_y, dy)
        max_y = max(max_y, dy + h)

    canvas_w = int(np.ceil(max_x - min_x))
    canvas_h = int(np.ceil(max_y - min_y))
    canvas_translation = np.array([-min_x, -min_y])

    canvas = np.zeros((canvas_h, canvas_w, 3), dtype=images[0].dtype)

    for i, img in enumerate(images):
        dx, dy = global_offsets[i]
        total_translation = np.array([dx, dy]) + canvas_translation
        M = np.float32([[1, 0, total_translation[0]], [0, 1, total_translation[1]]])
        warped_img = cv2.warpAffine(
            img,
            M,
            (canvas_w, canvas_h),
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=(0, 0, 0),
        )
        canvas = np.maximum(canvas, warped_img)

    return canvas


def stitch_folder(folder_path, output_path="mosaic.png"):
    """
    Loads images from a folder, stitches them as a row, and saves the mosaic.
    """
    try:
        images = load_images_sorted(folder_path)
        mosaic = stitch_row(images)
        if mosaic is not None:
            cv2.imwrite(output_path, mosaic)
    except Exception as e:
        raise RuntimeError(f"Stitching failed for folder {folder_path}: {e}") from e


if __name__ == "__main__":
    try:
        tile_dir = "tiles_example"
        os.makedirs(tile_dir, exist_ok=True)

        base_img = np.full((300, 400, 3), (255, 255, 255), dtype=np.uint8)
        cv2.rectangle(base_img, (125, 75), (275, 225), (255, 100, 0), -1)
        cv2.putText(
            base_img, "A", (175, 165), cv2.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 3
        )

        overlap = 100
        width = base_img.shape[1]
        tile1 = base_img[:, : width - overlap + 20]
        tile2 = base_img[:, overlap:]

        cv2.imwrite(os.path.join(tile_dir, "tile_01.png"), tile1)
        cv2.imwrite(os.path.join(tile_dir, "tile_02.png"), tile2)

        stitch_folder(tile_dir, "mosaic_example.png")

    except Exception as e:
        print(f"An error occurred during the example run: {e}")
