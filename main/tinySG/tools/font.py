import argparse
import struct
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont


def build(source, destination, size, characters):
    font = ImageFont.truetype(str(source), size)
    codes = sorted(set(map(ord, characters)))
    if not codes or codes[-1] > 255 or codes[-1] - codes[0] >= 255:
        raise ValueError("Select a nonempty character range within 0..255")
    first, count = codes[0], codes[-1] - codes[0] + 1
    bounds = font.getbbox(characters)
    height = bounds[3] - bounds[1]
    if not 0 < height < 256 or not 0 < size < 256:
        raise ValueError("The source raster size and height must fit in one byte")
    records, pixels = bytearray(), bytearray()
    base = 8 + count * 8
    for code in range(first, first + count):
        character = chr(code)
        advance = max(1, int(font.getlength(character) + 0.999))
        if advance > 255:
            raise ValueError("The glyph advance must fit in one byte")
        width = advance if code in codes else 0
        image = Image.new("L", (max(1, width), height))
        if width:
            ImageDraw.Draw(image).text((0, -bounds[1]), character, font=font, fill=255)
        data = image.tobytes() if width else b""
        packed = bytearray((len(data) + 1) // 2)
        for index, value in enumerate(data):
            packed[index // 2] |= ((value + 8) // 17) << (0 if index & 1 else 4)
        records += struct.pack(">IBBBB", base + len(pixels), width, advance, 0, 0)
        pixels += packed
    destination.write_bytes(b"TSF1" + bytes([first, count, height, size]) + records + pixels)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Convert a TTF/OTF font into a scalable tinySG A4 bitmap font")
    parser.add_argument("source", type=Path)
    parser.add_argument("destination", type=Path)
    parser.add_argument("--size", type=int, default=48)
    parser.add_argument("--characters", default="".join(map(chr, range(32, 127))))
    options = parser.parse_args()
    build(options.source, options.destination, options.size, options.characters)
