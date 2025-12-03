# tools/tflite_to_header.py
import sys
from pathlib import Path

def tflite_to_header(tflite_path, header_path, var_name="g_yolo_model"):
    data = Path(tflite_path).read_bytes()
    with open(header_path, "w", encoding="utf-8") as f:
        f.write("#pragma once\n#include <cstdint>\n\n")
        f.write(f"const unsigned char {var_name}[] = {{\n  ")
        for i, b in enumerate(data):
            f.write(f"0x{b:02x}, ")
            if (i + 1) % 12 == 0:
                f.write("\n  ")
        f.write("\n};\n")
        f.write(f"const unsigned int {var_name}_len = {len(data)};\n")
    print(f"written to {header_path} ({len(data)} bytes)")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("usage: python tflite_to_header.py model.tflite include/model.h")
        sys.exit(1)
    tflite_to_header(sys.argv[1], sys.argv[2])
