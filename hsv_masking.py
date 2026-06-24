"""
object_color_detector.py

Detects individual objects in an image, separates touching/overlapping
objects, and reports a per-object breakdown of the colors present
(not just a single averaged color).

Pipeline:
  1. Segment foreground vs background (works for light OR dark backgrounds).
  2. Separate touching objects using distance-transform + watershed,
     seeded by local maxima (works well for round/blob-like objects with
     consistent size; same-color objects touching with NO gap and NO
     curvature difference are a known hard case -- see README at bottom).
  3. For each separated object, classify every pixel inside its mask into
     a named color bucket and report the percentage breakdown.
  4. Label each object at its contour CENTROID (image moments), not the
     bounding-box corner.

SETUP (run once):
    pip install opencv-python numpy scipy
    # tested with opencv-python 4.13, numpy 2.4, scipy 1.17 -- any
    # reasonably recent version of each should work fine.

USAGE:
    # From the command line, quick check with default settings:
    python object_color_detector.py path/to/your_image.png

    # From your own script, with the parameters you actually need to set
    # per-photo (see detect_objects() docstring below for full details):
    from object_color_detector import detect_objects

    detections, output_path, bg_guess = detect_objects(
        "your_image.png",
        output_path="result.png",
        expected_object_fraction=0.26,   # measure this off YOUR photo:
                                          # crop one object, divide its
                                          # width by the full image width
        object_shape="general",          # "general" for cans/objects shot
                                          # upright or from the side;
                                          # "round_metallic" for a top-down
                                          # photo where you can see circular
                                          # can-tops; "side_by_side" for a
                                          # front-on row (worth comparing
                                          # against "general" -- whichever
                                          # gives cleaner separation on your
                                          # photo is the one to use)
    )
    for d in detections:
        print(d["dominant_color"], d["color_breakdown"], d["center"])

    The defaults (object_shape="auto", expected_object_fraction=0.12) were
    tuned for one specific dense top-down photo of canned drinks and will
    NOT be right for your photos -- always measure expected_object_fraction
    off the actual image you're using, and try both "general" and
    "round_metallic"/"side_by_side" to see which separates your objects
    more cleanly. There's no universal default because it depends on how
    the photo was shot.
"""

import cv2
import numpy as np
from scipy import ndimage as ndi


# ---------------------------------------------------------------------------
# Color classification
# ---------------------------------------------------------------------------
# Named HSV ranges (OpenCV hue 0-180). Order matters: checked top to bottom,
# first match wins. Tune these if your lighting/material is very different.
COLOR_RANGES = [
    # name        H low, H high   S low   V low   V high
    ("red",        [(0, 10), (170, 180)],  70,  60, 255),
    ("orange",     [(10, 22)],              70,  60, 255),
    ("yellow",     [(22, 35)],              70,  60, 255),
    ("green",      [(35, 85)],              50,  40, 255),
    ("blue",       [(85, 130)],             50,  40, 255),
    ("purple",     [(130, 155)],            40,  40, 255),
    ("pink",       [(155, 170)],            40,  60, 255),
]

def classify_pixel_block(hsv_pixels):
    """
    Classify a batch of HSV pixels (Nx3 array) into color-name buckets.
    Returns an array of string labels, one per pixel.

    Order of checks: grayscale family first (low saturation / extreme value),
    then hue-based buckets, then a brown special-case carved out of
    dark+desaturated orange/red.
    """
    h = hsv_pixels[:, 0].astype(np.int32)
    s = hsv_pixels[:, 1].astype(np.int32)
    v = hsv_pixels[:, 2].astype(np.int32)

    labels = np.full(len(hsv_pixels), "", dtype=object)

    # --- grayscale family: low saturation means "not very colorful" ---
    # Split into four bands by brightness: black (very dark), gray (mid),
    # silver (bright but not pure-white -- typical of reflective metal),
    # white (very bright / near pure white).
    low_sat = s < 35... （6 KB 剩餘）
