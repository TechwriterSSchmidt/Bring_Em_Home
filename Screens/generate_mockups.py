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
    
    # 1. Battery Icon (Vertical)
    # Body
    draw.rectangle((0, 4, 5, 13), outline=FG_COLOR)
    # Knob
    draw.rectangle((1, 2, 3, 4), fill=FG_COLOR)
    # Fill (85%)
    draw.rectangle((1, 6, 4, 12), fill=FG_COLOR)
    
    # Text
    draw.text((8, 2), "85%", font=font_small, fill=FG_COLOR)
    
    # 2. Compass
    comp_text = "C:Ok"
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
    font_title = get_font(14, bold=True)
    text = "GET EM HOME!"
    w = draw.textlength(text, font=font_title)
    draw.text(((WIDTH - w) / 2, 35), text, font=font_title, fill=FG_COLOR)
    
    # Subtext
    font_sub = get_font(12)
    text = "Returning..."
    w = draw.textlength(text, font=font_sub)
    draw.text(((WIDTH - w) / 2, 60), text, font=font_sub, fill=FG_COLOR)
    
    # Visualization: Arrow -> House
    # Center Y for graphics = 95
    base_y = 95
    
    # House (Right)
    hx = 85
    hy = base_y
    # Body
    draw.rectangle((hx, hy-8, hx+16, hy+8), outline=FG_COLOR)
    # Roof
    draw.polygon([(hx-2, hy-8), (hx+8, hy-18), (hx+18, hy-8)], outline=FG_COLOR)
    # Door
    draw.rectangle((hx+6, hy, hx+10, hy+8), fill=FG_COLOR)
    
    # Arrow (Left) pointing to House
    ax_start = 35
    ax_end = 75
    ay = base_y
    
    # Shaft
    draw.line((ax_start, ay, ax_end, ay), fill=FG_COLOR, width=2)
    # Head
    draw.polygon([(ax_end, ay), (ax_end-6, ay-6), (ax_end-6, ay+6)], fill=FG_COLOR)
    
    save_path = os.path.join(os.path.dirname(__file__), "mockup_sos.png")
    img.save(save_path)
    print(f"Generated {save_path} (Renamed content)")

def generate_confirm_home():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    draw_header(draw)
    
    font_title = get_font(12, bold=True)
    text = "SET HOME HERE?"
    w = draw.textlength(text, font=font_title)
    draw.text(((WIDTH - w) / 2, 30), text, font=font_title, fill=FG_COLOR)
    
    font_text = get_font(10)
    
    # Fake coords
    lat = "Lat: 48.13715"
    lon = "Lon: 11.57612"
    
    w = draw.textlength(lat, font=font_text)
    draw.text(((WIDTH - w) / 2, 50), lat, font=font_text, fill=FG_COLOR)
    
    w = draw.textlength(lon, font=font_text)
    draw.text(((WIDTH - w) / 2, 62), lon, font=font_text, fill=FG_COLOR)
    
    # Line
    draw.line((0, 75, 128, 75), fill=FG_COLOR)
    
    # Options
    font_opt = get_font(10, bold=True)
    draw.text((10, 95), "1x: YES", font=font_opt, fill=FG_COLOR)
    draw.text((10, 115), "2x: NO (Load)", font=font_opt, fill=FG_COLOR)
    
    save_path = os.path.join(os.path.dirname(__file__), "mockup_confirm.png")
    img.save(save_path)
    print(f"Generated {save_path}")

def generate_searching():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    draw_header(draw)
    
    # Text
    font_text = get_font(14, bold=True)
    text = "Searching SATs"
    w = draw.textlength(text, font=font_text)
    
    # Center
    start_x = (WIDTH - w) / 2
    y_text = 46 # Approx top for baseline at 60
    
    draw.text((start_x, y_text), text, font=font_text, fill=FG_COLOR)

    # Progress Bar
    bar_width = 64
    bar_height = 12
    bar_x = (WIDTH - bar_width) / 2
    bar_y = 75 # Matches C++ code (y=60 + 15)
    
    # Frame
    draw.rectangle((bar_x, bar_y, bar_x + bar_width, bar_y + bar_height), outline=FG_COLOR)
    
    # Fill (50%)
    fill_width = bar_width / 2
    draw.rectangle((bar_x + 2, bar_y + 2, bar_x + fill_width, bar_y + bar_height - 2), fill=FG_COLOR)
    
    save_path = os.path.join(os.path.dirname(__file__), "mockup_searching.png")
    img.save(save_path)
    print(f"Generated {save_path}")

def draw_arrow(draw, cx, cy, r, angle_deg):
    # Convert to radians
    # -90 to point up at 0 deg
    angle_rad = math.radians(angle_deg - 90)
    
    # Tip
    x1 = cx + r * math.cos(angle_rad)
    y1 = cy + r * math.sin(angle_rad)
    
    # Tail (indented)
    x2 = cx - (r * 0.3) * math.cos(angle_rad)
    y2 = cy - (r * 0.3) * math.sin(angle_rad)
    
    # Side 1 (Right)
    x3 = cx + (r * 0.35) * math.cos(angle_rad + math.pi/2)
    y3 = cy + (r * 0.35) * math.sin(angle_rad + math.pi/2)

    # Side 2 (Left)
    x4 = cx + (r * 0.35) * math.cos(angle_rad - math.pi/2)
    y4 = cy + (r * 0.35) * math.sin(angle_rad - math.pi/2)
    
    # Draw two triangles
    draw.polygon([(x1, y1), (x3, y3), (x2, y2)], fill=FG_COLOR)
    draw.polygon([(x1, y1), (x4, y4), (x2, y2)], fill=FG_COLOR)
    
    # Center Dot
    r_dot = 3
    draw.ellipse((cx-r_dot, cy-r_dot, cx+r_dot, cy+r_dot), fill=FG_COLOR)

def generate_breadcrumb():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    draw_header(draw)
    
    # Arrow pointing North-East (45 deg) relative to heading
    # Let's say we need to turn right
    draw_arrow(draw, WIDTH/2, 64, 30, 45)
    
    # Footer
    
    # Line 1: Next WP (Baseline 110)
    font_next = get_font(10) 
    y1 = 110 - 10
    draw.text((0, y1), "NEXT:", font=font_next, fill=FG_COLOR)
    
    next_text = "150m"
    w = draw.textlength(next_text, font=font_next)
    draw.text((WIDTH - w, y1), next_text, font=font_next, fill=FG_COLOR)
    
    # Line 2: Home (Baseline 125)
    font_lbl = get_font(12, bold=True)
    font_dist = get_font(14, bold=True)
    y2 = 125 - 14 # Use largest font height
    
    draw.text((0, y2), "HOME", font=font_lbl, fill=FG_COLOR)
    
    dist_text = "0.85 km"
    w = draw.textlength(dist_text, font=font_dist)
    draw.text((WIDTH - w, y2), dist_text, font=font_dist, fill=FG_COLOR)
    
    save_path = os.path.join(os.path.dirname(__file__), "mockup_breadcrumb.png")
    img.save(save_path)
    print(f"Generated {save_path}")

def generate_nav():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    draw_header(draw)
    
    # Arrow (Center)
    cx, cy = 64, 64
    r = 30
    
    # Draw Arrow (Pointing North/Up)
    # Tip
    draw.polygon([(cx, cy-r), (cx-10, cy+10), (cx, cy+5), (cx+10, cy+10)], outline=FG_COLOR, fill=FG_COLOR)
    
    # Cardinals (N, ., E, ., S, ., W, .)
    font_card = get_font(8)
    r_text = r + 14
    
    dirs = [("N", 0), (".", 45), ("E", 90), (".", 135), 
            ("S", 180), (".", 225), ("W", 270), (".", 315)]
            
    for label, angle in dirs:
        # 0 deg = North (Up) -> -90 deg in standard trig
        angle_rad = math.radians(angle - 90)
        
        tx = cx + r_text * math.cos(angle_rad)
        ty = cy + r_text * math.sin(angle_rad)
        
        if label == ".":
            # Draw 2x2 dot (4 pixels)
            # PIL rectangle is inclusive [x0, y0, x1, y1]
            draw.rectangle((tx-1, ty-1, tx, ty), fill=FG_COLOR)
        else:
            # Center text
            w = draw.textlength(label, font=font_card)
            h = 8 # approx height
            draw.text((tx - w/2, ty - h/2), label, font=font_card, fill=FG_COLOR)
    
    # Footer (Bottom)
    y_footer = 125 - 12 # Baseline approx
    
    # Label (Left)
    font_lbl = get_font(12, bold=True)
    draw.text((0, y_footer), "HOME", font=font_lbl, fill=FG_COLOR)
    
    # Distance (Right)
    font_dist = get_font(14, bold=True)
    text = "0.45 km"
    w = draw.textlength(text, font=font_dist)
    draw.text((WIDTH - w, y_footer), text, font=font_dist, fill=FG_COLOR)
    
    save_path = os.path.join(os.path.dirname(__file__), "mockup_nav.png")
    img.save(save_path)
    print(f"Generated {save_path}")

def generate_charging():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    # Title
    font_title = get_font(14, bold=True)
    text = "Loading battery..."
    w = draw.textlength(text, font=font_title)
    draw.text(((WIDTH - w) / 2, 30), text, font=font_title, fill=FG_COLOR)
    
    # Battery Icon Large
    # u8g2.drawFrame(44, 50, 40, 20); // Body
    # u8g2.drawBox(84, 56, 4, 8);     // Terminal
    draw.rectangle((44, 50, 84, 70), outline=FG_COLOR)
    draw.rectangle((84, 56, 88, 64), fill=FG_COLOR)
    
    # Animated Fill (3 bars)
    # u8g2.drawBox(46 + (i*9), 52, 7, 16);
    for i in range(3):
        x = 46 + (i * 9)
        draw.rectangle((x, 52, x + 7, 68), fill=FG_COLOR)
        
    # Estimated Time
    font_est = get_font(14, bold=True)
    text = "Est: 1.5h"
    w = draw.textlength(text, font=font_est)
    draw.text(((WIDTH - w) / 2, 90), text, font=font_est, fill=FG_COLOR)
        
    save_path = os.path.join(os.path.dirname(__file__), "mockup_charging.png")
    img.save(save_path)
    print(f"Generated {save_path}")

def generate_sos_countdown():
    img = create_image()
    draw = ImageDraw.Draw(img)
    
    # Title
    font_title = get_font(16, bold=True)
    text = "SOS MODE IN"
    w = draw.textlength(text, font=font_title)
    draw.text(((WIDTH - w) / 2, 20), text, font=font_title, fill=FG_COLOR)
    
    # Countdown
    font_huge = get_font(42, bold=True)
    text = "3"
    w = draw.textlength(text, font=font_huge)
    # Centered vertically at Y=45 (approx)
    draw.text(((WIDTH - w) / 2, 45), text, font=font_huge, fill=FG_COLOR)
    
    # Footer
    font_sub = get_font(10)
    text = "Press button to Cancel"
    w = draw.textlength(text, font=font_sub)
    draw.text(((WIDTH - w) / 2, 110), text, font=font_sub, fill=FG_COLOR)
    
    save_path = os.path.join(os.path.dirname(__file__), "mockup_sos_countdown.png")
    img.save(save_path)
    print(f"Generated {save_path}")

if __name__ == "__main__":
    generate_sos()
    generate_searching()
    generate_nav()
    generate_charging()
    generate_sos_countdown()
    generate_confirm_home()
    generate_breadcrumb()
    generate_sos_countdown()
    generate_confirm_home()
    generate_breadcrumb()
