#!/usr/bin/env bash
mkdir YoloV3
cd YoloV3
touch DARKNET
wget --no-check-certificate https://pjreddie.com/media/files/yolov3.weights -O deploy.weights
wget --no-check-certificate 'https://raw.githubusercontent.com/pjreddie/darknet/master/cfg/yolov3.cfg' -O deploy.cfg
wget --no-check-certificate 'https://raw.githubusercontent.com/pjreddie/darknet/master/data/coco.names' -O classes.txt
cd ..