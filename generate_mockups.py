from PIL import Image, ImageDraw, ImageFont
import math
import os

# Constants
WIDTH = 128
HEIGHT = 128
BG_COLOR = 0  # Black
FG_COLOR = 255 # White

def create_image():
    return Image.new('1', (WIDTH, HEIGHT), BG_COLOR)

def get_font(size, bold=False):
    # Try to load a default font, otherwise fallback
    try:
        # On Windows, Arial is usually available
        font_name = "arialbd.ttf" if bold else "arial.ttf"
        return ImageFont.truetype(font_name, size)
    except IOError:
        return ImageFont.load_default()

def draw_header(draw):
    # Top Line
    draw.line((0, 15, 128, 15), fill=FG_COLOR)
    
    font_small = get_font(10) # Approx 5x7
    
    # 1. Battery
    draw.text((0, 2), "Bat:85%", font=font_small, fill=FG_COLOR)
    
    # 2. Compass
    comp_text = "Comp:Ok"
    # Center calculation roughly
    w = draw.textlength(comp_text, font=font_small)
    draw.text(((128 - w) / 2, 2), comp_text, font=font_small, fill=FG_COLOR)
    
    # 3. SAT Bars
    # Draw 4 bars at right
    start_x = 128 - (4 * 4)
    bars = 3 # Example: 3 bars
    for i in range(4):
        h = (i + 1) * 2 + 2
        x = start_x + (i * 4)
        y = 14 - h
        if i < bars:
            draw.rectangle((x, y, x + 2, 14), fill=FG_COLOR)
        else:
            draw.rectangle((x, y, x + 2, 14), outline=FG_COLOR)

def generate_sos():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    # Title
    font_title = get_font(16, bold=True)
    text = "SOS ACTIVE"
    w = draw.textlength(text, font=font_title)
    draw.text(((WIDTH - w) / 2, 20), text, font=font_title, fill=FG_COLOR)
    
    # Countdown
    font_huge = get_font(42, bold=True)
    text = "118"
    w = draw.textlength(text, font=font_huge)
    draw.text(((WIDTH - w) / 2, 50), text, font=font_huge, fill=FG_COLOR)
    
    # Footer
    font_sub = get_font(10)
    text = "Sending Location..."
    w = draw.textlength(text, font=font_sub)
    draw.text(((WIDTH - w) / 2, 110), text, font=font_sub, fill=FG_COLOR)
    
    img.save("mockup_sos.png")
    print("Generated mockup_sos.png")

def generate_searching():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    draw_header(draw)
    
    # Text Animation
    font_text = get_font(14, bold=True)
    text = "Searching SATs"
    w = draw.textlength(text, font=font_text)
    
    # Center roughly
    total_w = w + 20 # Space for dots
    start_x = (WIDTH - total_w) / 2
    y = 64 - 7 # Center vertically
    
    draw.text((start_x, y), text, font=font_text, fill=FG_COLOR)
    
    # Dots
    draw.text((start_x + w, y), " . . .", font=font_text, fill=FG_COLOR)
    
    img.save("mockup_searching.png")
    print("Generated mockup_searching.png")

def generate_nav():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    draw_header(draw)
    
    # Mode Title (Removed in code, but let's keep clean)
    
    # Arrow (Center)
    cx, cy = 64, 64
    r = 35
    
    # Draw Arrow (Pointing North/Up)
    # Tip
    draw.polygon([(cx, cy-r), (cx-10, cy+10), (cx, cy+5), (cx+10, cy+10)], outline=FG_COLOR, fill=FG_COLOR)
    
    # Cardinals (N, E, S, W)
    font_card = get_font(10)
    r_text = r + 12
    
    # N (Top)
    draw.text((cx-3, cy-r_text-5), "N", font=font_card, fill=FG_COLOR)
    # E (Right)
    draw.text((cx+r_text, cy-5), "E", font=font_card, fill=FG_COLOR)
    # S (Bottom)
    draw.text((cx-3, cy+r_text-5), "S", font=font_card, fill=FG_COLOR)
    # W (Left)
    draw.text((cx-r_text-5, cy-5), "W", font=font_card, fill=FG_COLOR)
    
    # Distance
    font_dist = get_font(18, bold=True)
    text = "450 m"
    w = draw.textlength(text, font=font_dist)
    draw.text(((WIDTH - w) / 2, 90), text, font=font_dist, fill=FG_COLOR)
    
    # Label
    font_lbl = get_font(10)
    text = "HOME"
    w = draw.textlength(text, font=font_lbl)
    draw.text(((WIDTH - w) / 2, 115), text, font=font_lbl, fill=FG_COLOR)
    
    img.save("mockup_nav.png")
    print("Generated mockup_nav.png")

if __name__ == "__main__":
    generate_sos()
    generate_searching()
    generate_nav()
