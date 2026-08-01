# YOLO26 detection result for medicine-box verification.
# Published by verifier_node on /yolo/detection.

std_msgs/Header header

# Whether the target object (medicine box) was detected with sufficient confidence.
bool confirmed

# Object class name (e.g. "medicine_box").
string class_name

# Bounding box in pixel coordinates.
float32 x1
float32 y1
float32 x2
float32 y2

# Horizontal offset of bbox center from image center (pixels, positive = right).
float32 offset_x

# Vertical offset of bbox center from image center (pixels, positive = down).
float32 offset_y

# Detection confidence [0.0, 1.0].
float32 confidence

# Image dimensions at detection time.
uint16 image_width
uint16 image_height
