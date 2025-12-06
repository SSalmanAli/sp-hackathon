---
title: "Lesson 4.1: Vision-Language Integration"
sidebar_position: 1
description: "Combining computer vision and language models for robot perception"
---

# Lesson 4.1: Vision-Language Integration

## Context
This lesson focuses on integrating computer vision and language models to create multimodal AI systems for robot perception. Vision-Language models enable robots to understand and respond to visual queries using natural language, which is essential for human-robot interaction in Physical AI systems.

## What You'll Build
In this lesson, you'll create a robot system that responds to visual queries using language. You'll integrate computer vision models with language processing to enable the robot to identify objects, understand spatial relationships, and respond to natural language queries about its visual environment.

## Required Tools
- OpenCV
- Whisper (or similar speech recognition)
- Transformers library
- ROS 2 (Humble Hawksbill)
- vision_msgs package
- sensor_msgs package

## Important Concepts

### Vision-Language Models
AI models that can process both visual and textual information simultaneously to understand complex multimodal queries.

### Object Detection
Identifying and localizing objects within images with bounding boxes and class labels.

### Spatial Reasoning
Understanding the spatial relationships between objects in an environment.

### Natural Language Processing
Processing and understanding human language to extract meaning and intent.

## Implementation Steps

### Step 1: Set Up Vision System
1. Configure camera input and image processing pipeline
2. Implement object detection using pre-trained models
3. Set up spatial reasoning for object relationships

### Step 2: Integrate Language Processing
1. Set up speech recognition for natural language input
2. Configure language model for query understanding
3. Create mapping between language and visual concepts

### Step 3: Implement Multimodal Processing
1. Combine visual and language information
2. Create response generation system
3. Implement action selection based on multimodal input

### Step 4: Test and Validate
1. Test system with various visual-language queries
2. Validate accuracy of object detection and language understanding
3. Optimize performance and response time

## Code Example: Vision-Language Processing Node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from transformers import pipeline, AutoTokenizer, AutoModel
import torch

class VisionLanguageNode(Node):
    def __init__(self):
        super().__init__('vision_language_node')

        # Create subscribers and publishers
        self.image_sub = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)

        self.query_sub = self.create_subscription(
            String,
            'vision_query',
            self.query_callback,
            10)

        self.response_pub = self.create_publisher(String, 'vision_response', 10)

        # Initialize CV bridge
        self.bridge = CvBridge()

        # Initialize computer vision components
        self.object_detector = self.initialize_object_detection()

        # Initialize language processing
        self.qa_pipeline = pipeline("question-answering", model="distilbert-base-cased-distilled-squad")
        self.tokenizer = AutoTokenizer.from_pretrained("distilbert-base-cased-distilled-squad")
        self.text_encoder = AutoModel.from_pretrained("distilbert-base-cased-distilled-squad")

        # Store latest image for processing
        self.latest_image = None
        self.objects_in_scene = []

        self.get_logger().info('Vision-Language node initialized')

    def initialize_object_detection(self):
        """Initialize object detection model"""
        # Using OpenCV's DNN module with a pre-trained model
        # In practice, you might use YOLO, SSD, or other models
        net = cv2.dnn.readNetFromDarknet(
            '/path/to/yolo/yolov4.cfg',  # Replace with actual path
            '/path/to/yolo/yolov4.weights'  # Replace with actual path
        )
        return net

    def detect_objects(self, image):
        """Detect objects in the image using YOLO"""
        height, width = image.shape[:2]

        # Create blob from image
        blob = cv2.dnn.blobFromImage(image, 1/255.0, (416, 416), swapRB=True, crop=False)
        self.object_detector.setInput(blob)

        # Run forward pass
        layer_names = self.object_detector.getLayerNames()
        output_layers = [layer_names[i[0] - 1] for i in self.object_detector.getUnconnectedOutLayers()]
        outputs = self.object_detector.forward(output_layers)

        # Process detections
        boxes = []
        confidences = []
        class_ids = []

        for output in outputs:
            for detection in output:
                scores = detection[5:]
                class_id = np.argmax(scores)
                confidence = scores[class_id]

                if confidence > 0.5:  # Threshold
                    center_x = int(detection[0] * width)
                    center_y = int(detection[1] * height)
                    w = int(detection[2] * width)
                    h = int(detection[3] * height)

                    x = int(center_x - w / 2)
                    y = int(center_y - h / 2)

                    boxes.append([x, y, w, h])
                    confidences.append(float(confidence))
                    class_ids.append(class_id)

        # Apply non-maximum suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.4)

        # Class names for COCO dataset
        class_names = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck",
                      "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
                      "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
                      "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
                      "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
                      "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
                      "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
                      "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa",
                      "pottedplant", "bed", "diningtable", "toilet", "tvmonitor", "laptop", "mouse",
                      "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
                      "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier", "toothbrush"]

        # Store detected objects with their properties
        detected_objects = []
        if len(indices) > 0:
            for i in indices.flatten():
                x, y, w, h = boxes[i]
                label = str(class_names[class_ids[i]])
                confidence = confidences[i]

                # Calculate center and store object info
                center_x = x + w // 2
                center_y = y + h // 2

                obj_info = {
                    'label': label,
                    'confidence': confidence,
                    'bbox': (x, y, w, h),
                    'center': (center_x, center_y),
                    'area': w * h
                }
                detected_objects.append(obj_info)

        return detected_objects

    def spatial_relationships(self, objects):
        """Determine spatial relationships between objects"""
        relationships = []

        for i, obj1 in enumerate(objects):
            for j, obj2 in enumerate(objects):
                if i != j:
                    # Calculate distance between object centers
                    dx = obj1['center'][0] - obj2['center'][0]
                    dy = obj1['center'][1] - obj2['center'][1]
                    distance = np.sqrt(dx*dx + dy*dy)

                    # Determine relative position
                    if abs(dx) > abs(dy):  # Horizontal relationship dominates
                        if dx > 0:
                            direction = "to the right of"
                        else:
                            direction = "to the left of"
                    else:  # Vertical relationship dominates
                        if dy > 0:
                            direction = "below"
                        else:
                            direction = "above"

                    # Only add if objects are reasonably close
                    if distance < 300:  # Adjust threshold as needed
                        relationship = {
                            'subject': obj1['label'],
                            'relationship': direction,
                            'object': obj2['label'],
                            'distance': distance
                        }
                        relationships.append(relationship)

        return relationships

    def image_callback(self, msg):
        """Process incoming image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store latest image
            self.latest_image = cv_image.copy()

            # Detect objects in the image
            self.objects_in_scene = self.detect_objects(cv_image)

            # Log detected objects
            object_labels = [obj['label'] for obj in self.objects_in_scene]
            self.get_logger().info(f'Detected objects: {list(set(object_labels))}')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def query_callback(self, msg):
        """Process vision query"""
        query = msg.data
        self.get_logger().info(f'Received query: {query}')

        if self.latest_image is None or not self.objects_in_scene:
            response = "No image data available. Please ensure camera is working."
        else:
            response = self.process_vision_query(query)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

    def process_vision_query(self, query):
        """Process a vision query against the current scene"""
        query_lower = query.lower()

        # Handle different types of queries
        if "how many" in query_lower or "count" in query_lower:
            # Count objects of specific type
            for obj_type in ["person", "car", "chair", "table", "bottle", "cup"]:
                if obj_type in query_lower:
                    count = len([obj for obj in self.objects_in_scene if obj['label'] == obj_type])
                    return f"I see {count} {obj_type}{'s' if count != 1 else ''} in the scene."

            # Count all objects
            total_objects = len(self.objects_in_scene)
            return f"I see {total_objects} objects in total."

        elif "where is" in query_lower or "location" in query_lower or "position" in query_lower:
            # Find location of specific object
            for obj in self.objects_in_scene:
                if obj['label'] in query_lower:
                    x, y, w, h = obj['bbox']
                    center_x, center_y = obj['center']

                    # Determine rough position in image
                    height, width = self.latest_image.shape[:2]
                    if center_x < width/3:
                        h_pos = "on the left"
                    elif center_x < 2*width/3:
                        h_pos = "in the center"
                    else:
                        h_pos = "on the right"

                    if center_y < height/3:
                        v_pos = "at the top"
                    elif center_y < 2*height/3:
                        v_pos = "in the middle"
                    else:
                        v_pos = "at the bottom"

                    return f"The {obj['label']} is located {h_pos} {v_pos} of the image."

            return f"I don't see a {query.split()[-1]} in the scene."

        elif "color" in query_lower:
            # This would require more sophisticated color detection
            # For now, return a general response
            return "I can detect object types but not specific colors in this implementation."

        elif "spatial" in query_lower or "relationship" in query_lower or "next to" in query_lower or "near" in query_lower:
            # Analyze spatial relationships
            relationships = self.spatial_relationships(self.objects_in_scene)
            if relationships:
                # Return a few relationships
                rel_strs = []
                for rel in relationships[:5]:  # Limit to first 5 relationships
                    rel_strs.append(f"{rel['subject']} is {rel['relationship']} {rel['object']}")
                return f"Here are some spatial relationships: {', '.join(rel_strs)}"
            else:
                return "I don't detect clear spatial relationships between objects in the scene."

        else:
            # General question answering about the scene
            object_names = [obj['label'] for obj in self.objects_in_scene]
            if object_names:
                return f"The scene contains: {', '.join(set(object_names))}. How can I help you with these objects?"
            else:
                return "I don't detect any objects in the current scene."

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    vision_language_node = VisionLanguageNode()

    try:
        rclpy.spin(vision_language_node)
    except KeyboardInterrupt:
        pass
    finally:
        vision_language_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Code Example: Speech Recognition Integration

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import AudioData
import speech_recognition as sr
import threading
import queue

class SpeechRecognitionNode(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')

        # Create publisher for recognized text
        self.text_pub = self.create_publisher(String, 'recognized_text', 10)

        # Initialize speech recognizer
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()

        # Set energy threshold for ambient noise
        with self.microphone as source:
            self.recognizer.adjust_for_ambient_noise(source)

        # Start speech recognition in a separate thread
        self.recognition_thread = threading.Thread(target=self.speech_recognition_loop)
        self.recognition_thread.daemon = True
        self.recognition_thread.start()

        self.get_logger().info('Speech recognition node initialized')

    def speech_recognition_loop(self):
        """Continuously listen for speech and recognize it"""
        while rclpy.ok():
            try:
                with self.microphone as source:
                    self.get_logger().info('Listening...')
                    audio = self.recognizer.listen(source, timeout=5, phrase_time_limit=10)

                # Recognize speech using Google Web Speech API
                text = self.recognizer.recognize_google(audio)
                self.get_logger().info(f'Recognized: {text}')

                # Publish recognized text
                text_msg = String()
                text_msg.data = text
                self.text_pub.publish(text_msg)

            except sr.WaitTimeoutError:
                # No speech detected within timeout, continue listening
                continue
            except sr.UnknownValueError:
                self.get_logger().warn('Could not understand audio')
            except sr.RequestError as e:
                self.get_logger().error(f'Error with speech recognition service: {e}')
            except Exception as e:
                self.get_logger().error(f'Unexpected error in speech recognition: {e}')

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    speech_node = SpeechRecognitionNode()

    try:
        rclpy.spin(speech_node)
    except KeyboardInterrupt:
        pass
    finally:
        speech_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Code Example: Vision Query Processor

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from vision_msgs.msg import Detection2DArray, ObjectHypothesisWithPose
import json

class VisionQueryProcessor(Node):
    def __init__(self):
        super().__init__('vision_query_processor')

        # Create subscribers
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            'detections',
            self.detection_callback,
            10)

        self.query_sub = self.create_subscription(
            String,
            'natural_query',
            self.query_callback,
            10)

        # Create publisher
        self.response_pub = self.create_publisher(String, 'query_response', 10)

        # Store detections
        self.latest_detections = []

        self.get_logger().info('Vision query processor initialized')

    def detection_callback(self, msg):
        """Store latest detections"""
        self.latest_detections = []
        for detection in msg.detections:
            obj_info = {
                'id': detection.results[0].id if detection.results else '',
                'score': detection.results[0].score if detection.results else 0.0,
                'bbox': {
                    'x': detection.bbox.center.x,
                    'y': detection.bbox.center.y,
                    'width': detection.bbox.size_x,
                    'height': detection.bbox.size_y
                }
            }
            self.latest_detections.append(obj_info)

    def query_callback(self, msg):
        """Process natural language query about vision data"""
        query = msg.data
        self.get_logger().info(f'Processing query: {query}')

        response = self.process_query(query)

        # Publish response
        response_msg = String()
        response_msg.data = response
        self.response_pub.publish(response_msg)

    def process_query(self, query):
        """Process the query against stored detections"""
        if not self.latest_detections:
            return "No visual data available to answer your query."

        query_lower = query.lower()

        if "count" in query_lower or "many" in query_lower:
            return f"I detect {len(self.latest_detections)} objects in the scene."

        # For more complex queries, you would integrate with NLP models
        # This is a simplified example
        return f"I have processed your query about the scene. Detected {len(self.latest_detections)} objects."

def main(args=None):
    rclpy.init(args=args)

    processor = VisionQueryProcessor()

    try:
        rclpy.spin(processor)
    except KeyboardInterrupt:
        pass
    finally:
        processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Exercises

1. **Advanced Object Recognition**: Implement more sophisticated object recognition using deep learning models like YOLO or Mask R-CNN
2. **Spatial Reasoning**: Enhance the system to understand complex spatial relationships and directions
3. **Multimodal Learning**: Create a system that learns associations between visual concepts and language through interaction

## Troubleshooting Tips

- Ensure camera calibration is accurate for proper spatial reasoning
- Verify that the object detection model is properly trained for your environment
- Check that speech recognition is working in your acoustic environment
- Monitor computational resources as vision-language processing can be demanding
- Validate that TF frames are properly connected between camera and robot base

## Learning Outcomes
By completing this lesson, you will understand:
- How to integrate computer vision and language processing systems
- How to implement object detection and spatial reasoning
- How to process natural language queries about visual scenes
- How to create multimodal AI systems for robot perception