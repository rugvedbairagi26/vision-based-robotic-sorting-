# Computer Vision – Custom Colored Cube Detection

## Overview

This module focuses on the computer vision and object detection component of the Vision-Based Robotic Sorting System.

A custom dataset was created using three types of colored cubes: red, green, and yellow. The images were annotated with bounding boxes and used to train a YOLO object detection model.

The objective is to develop a vision system capable of detecting and classifying colored cubes, which can eventually provide object information to the robotic arm for autonomous sorting operations.

## Project Objectives

- Create a custom dataset containing red, green, and yellow cubes.
- Annotate images for object detection.
- Train a YOLO model using the custom dataset.
- Evaluate the trained model using standard object detection metrics.
- Detect and classify colored cubes from input images.
- Develop the perception component for future robotic integration.

## Technologies Used

- Python 3.13
- YOLO (Ultralytics 8.4.172)
- PyTorch 2.11.0
- OpenCV
- NumPy
- Matplotlib
- Google Colab
- NVIDIA Tesla T4 GPU

## Dataset

A custom object detection dataset was created specifically for this project.

**Object classes:**

1. Green Cube
2. Red Cube
3. Yellow Cube

The dataset contains annotated images of the three cube classes and was used for training and evaluating the YOLO model.

The dataset is organized into training, validation, and testing subsets.

## Model Training

The custom dataset was used to train a YOLO object detection model.

### Training Environment

| Parameter | Configuration |
|---|---|
| Framework | Ultralytics |
| Ultralytics Version | 8.4.172 |
| Python | 3.13.15 |
| PyTorch | 2.11.0+cu130 |
| GPU | NVIDIA Tesla T4 |
| Model Parameters | 3,006,233 |
| Model GFLOPs | 8.1 |

## Model Evaluation Results

The trained model was evaluated on a test set containing 30 images and 32 object instances.

### Overall Performance

| Metric | Result |
|---|---:|
| Precision | 0.988 |
| Recall | 1.000 |
| mAP@50 | 0.995 |
| mAP@50-95 | 0.668 |

### Class-wise Performance

| Class | Images | Instances | Precision | Recall | mAP@50 | mAP@50-95 |
|---|---:|---:|---:|---:|---:|---:|
| Green | 12 | 12 | 0.987 | 1.000 | 0.995 | 0.697 |
| Red | 12 | 12 | 0.986 | 1.000 | 0.995 | 0.629 |
| Yellow | 8 | 8 | 0.992 | 1.000 | 0.995 | 0.679 |
| **Overall** | **30** | **32** | **0.988** | **1.000** | **0.995** | **0.668** |

### Inference Speed

| Operation | Time per Image |
|---|---:|
| Preprocessing | 11.7 ms |
| Inference | 19.3 ms |
| Postprocessing | 9.1 ms |

## Repository Structure

```text
perception/
│
├── README.md
│
├── dataset/
│   ├── README.md
│   ├── dataset.yaml
│   └── sample_images/
│
├── notebooks/
│   └── colored_cube_yolo_training.ipynb
│
├── src/
│   └── detect_cubes.py
│
├── weights/
│   └── README.md
│
└── results/
    ├── metrics.md
    ├── training_curves.png
    └── predictions/
```

## Project Workflow

1. Collect images of red, green, and yellow cubes.
2. Create and annotate a custom object detection dataset.
3. Organize the dataset into training, validation, and testing subsets.
4. Configure and train the YOLO object detection model.
5. Evaluate the trained model using precision, recall, and mAP.
6. Perform object detection on test images.
7. Analyze the predictions and model performance.

## Integration with Robotic Manipulation

The perception module is intended to identify the color and image location of cubes.

The detected class labels and bounding box coordinates can serve as inputs for future object localization, camera-to-robot coordinate transformation, grasp planning, and robotic manipulation.

Integration with the complete physical robotic sorting system remains future work.

## Future Improvements

- Test detection using a live camera feed.
- Evaluate performance under different lighting conditions.
- Improve object localization accuracy.
- Estimate object positions in robot coordinate frames.
- Integrate the perception system with ROS 2 and MoveIt 2.
- Perform real-world autonomous color-based sorting.

## Project Status

The custom dataset has been created and annotated, and a YOLO object detection model has been trained and evaluated.

The model achieved an overall mAP@50 of 0.995 on the reported 30-image evaluation set.

The next stage is to connect the perception system with robotic manipulation for complete autonomous sorting.


