CamX Application Documentation
Overview
CamX is an intelligent camera application designed for automated object tracking using a motorized tripod. It leverages state-of-the-art computer vision (YOLOv8) to identify subjects in real-time and control hardware via network commands.
Architecture
1. Computer Vision Layer (YOLOv8)
The app uses YOLOv8 (You Only Look Once) implemented via TensorFlow Lite.
?
Inference: The model processes 640x640 images from the camera stream.
?
Detection: It identifies common object classes (people, vehicles, animals, etc.).
?
Logic: The YoloDetector.kt class handles preprocessing (scaling and normalization), inference, and post-processing (Non-Maximum Suppression).
2. Camera Layer (CameraX)
Built on the modern Android CameraX Jetpack library.
?
Preview: Real-time viewfinder for the user.
?
Image Analysis: Feeds frames to the YOLO engine.
?
Capture: Dedicated pipelines for high-resolution JPEGs and MP4 video recording with audio.
3. Hardware Control (UDP)
The app calculates the horizontal "Pan Angle" based on the subject's position in the frame.
?
Center: 90� (Neutral).
?
Range: 0� to 180�.
?
Communication: Sends UDP packets to an ESP32 microcontroller at 10.179.76.141.
Features
?
Object Tracking: Automatic detection and following of the most prominent subject.
?
Tracking Toggle: Turn tracking on or off to use the app as a standard camera.
?
Flip Camera: Support for both Front and Rear cameras.
?
Flash Control: On, Off, and Auto modes.
?
Video Recording: Save high-quality videos directly to the Gallery.
?
Photo Capture: Snap photos while tracking.
How to Install the YOLO Model
To ensure the best performance, you should provide an official YOLOv8 model file:
1.
Download: Obtain a yolov8n_float32.tflite model from a trusted source (like Ultralytics).
2.
Rename: Rename the file to yolov8n.tflite.
3.
Place: Copy the file into the app/src/main/assets/ directory of this project.
4.
Labels: If you have a labels.txt file (one class name per line), place it in the same folder.
Technical Specifications
?
Operating System: Android 8.0 (API 26) or higher.
?
Compatibility: 16 KB Page Size compatible (Android 15+ ready).
?
Libraries:
?
androidx.camera:camera-video:1.4.1
?
org.tensorflow:tensorflow-lite:2.16.1
