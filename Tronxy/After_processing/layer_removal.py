import re
import argparse

def filter_odd_layers(input_path: str, output_path: str=None):
    """
    Reads a G-code file, removes all layer blocks with odd layer numbers,
    and writes the result to output_path (or overwrites the input file if none specified).

    A layer block is defined as the lines from a ';MESH:NONMESH' marker
    up to but not including the next ';MESH:NONMESH' or the end code marker '; XY-3 End Code'.
    The layer number is parsed from a ';LAYER:(\d+)' line within the block.
    """
    # Read all lines
    with open(input_path, 'r') as f:
        lines = f.readlines()

    # Find indices of all block-start markers and the end-code marker
    mesh_markers = [i for i, ln in enumerate(lines) if ln.strip().startswith(';MESH:NONMESH')]
    end_code_idx = next((i for i, ln in enumerate(lines) if ln.strip().startswith('; XY-3 End Code')), len(lines))

    # Build ranges of blocks to remove: list of (start, end)
    remove_ranges = []

    # Append sentinel end index for easier looping
    boundaries = mesh_markers + [end_code_idx]

    for start_idx, next_idx in zip(boundaries, boundaries[1:]):
        block = lines[start_idx:next_idx]
        # Find layer number in this block
        layer_num = None
        for ln in block:
            m = re.match(r';LAYER:(\d+)', ln.strip())
            if m:
                layer_num = int(m.group(1))
                break
        # If no layer number found, skip removal
        if layer_num is None:
            continue
        # If odd, mark this block for removal
        if layer_num % 2 == 1:
            remove_ranges.append((start_idx, next_idx))

    # Remove blocks in reverse order to not mess up indices
    for start, end in sorted(remove_ranges, reverse=True):
        del lines[start:end]

    # Write output
    out_path = output_path or input_path
    with open(out_path, 'w') as f:
        f.writelines(lines)

    print(f"Filtered G-code written to: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Filter out odd-numbered layers from a G-code file.")
    parser.add_argument('input', help="Path to the input G-code file.")
    parser.add_argument('-o', '--output', help="Path to write the filtered G-code. Defaults to overwrite input.")
    args = parser.parse_args()

    filter_odd_layers(args.input, args.output)

if __name__ == '__main__':
    main()
