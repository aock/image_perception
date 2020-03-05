#!/usr/bin/env bash
mkdir YoloV3Tiny
cd YoloV3Tiny
# DARKNET flag file
touch DARKNET
wget --no-check-certificate 'https://pjreddie.com/media/files/yolov3-tiny.weights' -O deploy.weights
wget --no-check-certificate 'https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3-tiny.cfg' -O deploy.cfg
# classes
wget --no-check-certificate 'https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names' -O classes.txt
cd ..
