# Image Perception in ROS (WIP)

## Package Description
This package deals with object detection on images on mobile autonomous platforms. It provides a collection of various well known object detection artificial neural network models, like YoloV3 or MobileNetSSD. The main node has the ability to load different object detection models from frameworks like Caffee, Tensorflow or Darknet. OpenCV is used as Wrapper for inference. The inference can executed on CPU or GPU depending on your OpenCV installation. Additionally, this package provides a node, which tracks the found objects and gives them an unique ID.

## Dependencies
- Ros
- Boost
- OpenCV 3.4.1 (Minimum)

## Usage

### Download and Installation
```console
foo@bar:~/catkin_ws/src$ git clone https://github.com/aock/image_perception.git
foo@bar:~/catkin_ws/src$ cd ..
foo@bar:~/catkin_ws$ catkin_make
```

### Subpackages
- image_object_detection
  - find objects with a given dnn model
  - example: image_object_detection.launch
- image_object_tracking
  - track objects found by image_object_detection_node
  - example: image_object_tracking.launch
- image_perception_vizualization
  - vizualizing raw labels and tracked objects
  - TODO
- image_perception_msgs
  - Any message definition and conversion

## Image Object Detection (image_object_detection)

- Subscribes to: /camera/image_raw
- Publishes on: /image_object_detection_node/labels

### Download Model

#### YoloV3Tiny:
```console
foo@bar:~/catkin_ws$ roscd image_object_detection
foo@bar:~/catkin_ws/src/image_perception/image_object_detection$ cd models
foo@bar:~/catkin_ws/src/image_perception/image_object_detection/models$ . downloadYoloV3Tiny.sh
```

#### MobileNetSSD:
```console
foo@bar:~/catkin_ws$ roscd image_object_detection
foo@bar:~/catkin_ws/src/image_perception/image_object_detection$ cd models
foo@bar:~/catkin_ws/src/image_perception/image_object_detection/models$ . downloadMobileNetSSD.sh
```

### Start
```console
foo@bar:~/catkin_ws$ roslaunch image_object_detection image_object_detection.launch
```
The main node generates a topic "/image_object_detection_node/labels" containing all found objects with object ID and name.
If enabled the node publishes an image with the found objects on the topic "/image_object_detection_node/labeled_image".

### Parameters
Each model has individual but same named parameters. Have a look at the 'image_object_detection.yaml' file in the 'config' directory. Parameters:

#### Global Parameters

- modelName
  - string
  - Name of the model (Directory name)
- publishImages
  - bool
  - Publish an image with drawed rectangles and labels (disable for higher performance)
 
#### Individual Parameters:
- scale
  - float
  - scale of the input image of pixels in range 0-255. For Example: For range 0-1 you have to set 0.00392 as scale
- mean
  - float
  - Mean added to input image pixels
- swapRB
  - bool
  - swap red and blue channel
- inputSize
  - vector (size 2)
  - Set the input layer size
- confidenceThreshold
  - float
  - Keep Objects with a confidence higher than confidenceThreshold


## Tutorial
### Add Model
The "models" directory of the package contains all models in a specific format. Each model is represented as a folder with the model name e.g. "YoloV3Tiny". The folder contains 4 files:

- classes.txt 
  - Class names listed in a text file
- DARKNET
  - Flag file which indicates the framework
  - DARKNET, CAFFE, TENSORFLOW
- deploy.cfg
  - config file of model
  - CAFFE: .prototxt, DARKNET: .cfg
- deploy.weights
  - trained weights of the model
  - CAFFE: .caffemodel, DARKNET: .weights
  
Please feel free to provide a shell script for models that are working with this package.



