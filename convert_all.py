import os
from PIL import Image

def generate_c_array(img_path, var_name):
    img = Image.open(img_path).convert("RGBA")
    w, h = img.size
    
    # We will generate LV_IMG_CF_TRUE_COLOR_ALPHA
    # For LVGL 8 with 16-bit color, the format is RGB565 + 8-bit Alpha = 3 bytes per pixel
    data = []
    for y in range(h):
        for x in range(w):
            r, g, b, a = img.getpixel((x, y))
            # rgb565
            c = ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)
            # LVGL expects little endian or big endian? Depending on swap config.
            # Usually LV_COLOR_16_SWAP is 1 for SPI screens.
            c_swapped = ((c >> 8) & 0xFF) | ((c & 0xFF) << 8)
            # data order for LV_IMG_CF_TRUE_COLOR_ALPHA: color_byte1, color_byte2, alpha
            data.extend([c_swapped & 0xFF, (c_swapped >> 8) & 0xFF, a])
            
    c_out = f"const uint8_t {var_name}_map[] = {{\n"
    for i in range(0, len(data), 12):
        chunk = data[i:i+12]
        c_out += "  " + ", ".join(f"0x{b:02x}" for b in chunk) + ",\n"
    c_out += "};\n\n"
    
    c_out += f"""
const lv_img_dsc_t {var_name} = {{
  .header.always_zero = 0,
  .header.w = {w},
  .header.h = {h},
  .data_size = {len(data)},
  .header.cf = LV_IMG_CF_TRUE_COLOR_ALPHA,
  .data = {var_name}_map,
}};
"""
    return c_out

def main():
    img_dir = "images"
    files = [f for f in os.listdir(img_dir) if f.endswith(".png")]
    
    c_code = "#include \"lvgl.h\"\n\n"
    h_code = "#include \"lvgl.h\"\n\n"
    
    for f in files:
        path = os.path.join(img_dir, f)
        var_name = "img_" + f.replace(".png", "")
        print(f"Converting {path} -> {var_name}")
        c_code += generate_c_array(path, var_name)
        h_code += f"extern const lv_img_dsc_t {var_name};\n"
        
    with open("main/machi_images.c", "w") as fc:
        fc.write(c_code)
    with open("main/machi_images.h", "w") as fh:
        fh.write(h_code)

if __name__ == "__main__":
    main()
