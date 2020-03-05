#!/usr/bin/env bash
mkdir FasterRCNNInceptionV2
cd FasterRCNNInceptionV2
touch TENSORFLOW
# weights
wget --no-check-certificate 'http://download.tensorflow.org/models/object_detection/faster_rcnn_inception_v2_coco_2018_01_28.tar.gz'
tar -xvzf faster_rcnn_inception_v2_coco_2018_01_28.tar.gz faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb
mv faster_rcnn_inception_v2_coco_2018_01_28/frozen_inference_graph.pb deploy.pb
rm -r faster_rcnn_inception_v2_coco_2018_01_28*
wget --no-check-certificate 'https://raw.githubusercontent.com/opencv/opencv_extra/master/testdata/dnn/faster_rcnn_inception_v2_coco_2018_01_28.pbtxt' -O deploy.pbtxt
wget --no-check-certificate 'https://raw.githubusercontent.com/opencv/opencv/master/samples/data/dnn/object_detection_classes_coco.txt' -O classes.txt
cd ..