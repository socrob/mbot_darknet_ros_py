#!/bin/bash

# TODO set WEIGHT_DIR with rospack find
WEIGHT_DIR=$ROS_WORKSPACE/mbot_yolo/darknet_ros_py/ros/binaries/weights/
WEIGHT_FILE=yolov3.weights

if [ ! -f $WEIGHT_DIR/$WEIGHT_FILE ]; then
    
    echo "Darknet weight files not found. Downloading...";
    
    # Download pretrained weights from darknet yolo (https://pjreddie.com/darknet/yolo/)
    mkdir $WEIGHT_DIR
    cd $WEIGHT_DIR
    wget https://pjreddie.com/media/files/$WEIGHT_FILE
    
else
    
    echo "Darknet weight files found. ";
    
fi

