#!/usr/bin/env bash
mkdir MobileNetSSD
cd MobileNetSSD
# CAFFE flag file
touch CAFFE
# model and config
wget --no-check-certificate 'https://raw.githubusercontent.com/chuanqi305/MobileNet-SSD/master/voc/MobileNetSSD_deploy.prototxt' -O deploy.prototxt
wget --no-check-certificate 'https://docs.google.com/uc?export=download&id=0B3gersZ2cHIxRm5PMWRoTkdHdHc' -O deploy.caffemodel
# classes to classes.txt

echo "background" > classes.txt
echo "aeroplane" >> classes.txt
echo "bicycle" >> classes.txt
echo "bird" >> classes.txt
echo "boat" >> classes.txt
echo "bottle" >> classes.txt
echo "bus" >> classes.txt
echo "car" >> classes.txt
echo "cat" >> classes.txt
echo "chair" >> classes.txt
echo "cow" >> classes.txt
echo "diningtable" >> classes.txt
echo "dog" >> classes.txt
echo "horse" >> classes.txt
echo "motorbike" >> classes.txt
echo "person" >> classes.txt
echo "pottedplant" >> classes.txt
echo "sheep" >> classes.txt
echo "sofa" >> classes.txt
echo "train" >> classes.txt
echo "tvmonitor" >> classes.txt

cd ..
