import numpy as np

from ultralytics import YOLO
from ultralytics.models.yolo.yoloe import YOLOEVPSegPredictor

# Initialize model
model = YOLO("/home/duo/yoloe26/yoloe-26x-seg.pt")

# Define visual prompts using bounding boxes and their corresponding class IDs.
# Each box highlights an example of the object you want the model to detect.
visual_prompts = dict(
    bboxes=np.array(
        [
            [221.52, 405.8, 344.98, 857.54],  # Box enclosing person
            [120, 425, 160, 445],  # Box enclosing glasses
        ],
    ),
    cls=np.array(
        [
            0,  # ID to be assigned for person
            1,  # ID to be assigned for glasses
        ]
    ),
)

# Run inference on an image, using the provided visual prompts as guidance
results = model.predict(
    "ultralytics/assets/bus.jpg",
    visual_prompts=visual_prompts,
    predictor=YOLOEVPSegPredictor,
)

# Show results
results[0].show()