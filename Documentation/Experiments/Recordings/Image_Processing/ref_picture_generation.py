import random

from PIL import Image, ImageDraw

# --- Tunable Parameters ---
IMAGE_WIDTH = 2457
IMAGE_HEIGHT = 3207
DOT_DENSITY = 0.015  # Fraction of pixels to be turned into dots
LINE_DENSITY = 0.06  # Fraction of the image area to be covered by lines
DOT_SIZE = 2
LINE_WIDTH = 2
BACKGROUND_COLOR = (0, 0, 0)
ITEM_COLOR = (0, 93, 80)
OUTPUT_FILENAME = "reference_image.png"


def generate_reference_image():
    """
    Generates a reference image with random dots and lines.
    """
    # Create a new black image
    image = Image.new("RGB", (IMAGE_WIDTH, IMAGE_HEIGHT), BACKGROUND_COLOR)
    draw = ImageDraw.Draw(image)

    # --- Generate Dots ---
    num_dots = int(IMAGE_WIDTH * IMAGE_HEIGHT * DOT_DENSITY)
    for _ in range(num_dots):
        x = random.randint(0, IMAGE_WIDTH - DOT_SIZE)
        y = random.randint(0, IMAGE_HEIGHT - DOT_SIZE)
        draw.rectangle([x, y, x + DOT_SIZE - 1, y + DOT_SIZE - 1], fill=ITEM_COLOR)

    # --- Generate Lines ---
    # Heuristic for number of lines based on density
    num_lines = int(
        (IMAGE_WIDTH * IMAGE_HEIGHT * LINE_DENSITY)
        / (max(IMAGE_WIDTH, IMAGE_HEIGHT) * 0.1)
    )  # Approximate average line length
    for _ in range(num_lines):
        x1 = random.randint(0, IMAGE_WIDTH)
        y1 = random.randint(0, IMAGE_HEIGHT)
        # Make lines tend to be of a certain length
        x2 = x1 + random.randint(-int(IMAGE_WIDTH * 0.01), int(IMAGE_WIDTH * 0.1))
        y2 = y1 + random.randint(-int(IMAGE_HEIGHT * 0.01), int(IMAGE_HEIGHT * 0.1))

        # Clamp coordinates to be within image bounds
        x2 = max(0, min(IMAGE_WIDTH, x2))
        y2 = max(0, min(IMAGE_HEIGHT, y2))

        draw.line([x1, y1, x2, y2], fill=ITEM_COLOR, width=LINE_WIDTH)

    # Save the image
    image.save(OUTPUT_FILENAME)
    print(f"Image saved as {OUTPUT_FILENAME}")
    print(f"  - DOT_DENSITY: {DOT_DENSITY}")
    print(f"  - LINE_DENSITY: {LINE_DENSITY}")
    print(f"  - DOT_SIZE: {DOT_SIZE}")
    print(f"  - LINE_WIDTH: {LINE_WIDTH}")


if __name__ == "__main__":
    generate_reference_image()
