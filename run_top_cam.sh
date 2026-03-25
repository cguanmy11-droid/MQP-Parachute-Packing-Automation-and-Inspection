#!/bin/bash
BASE=/home/zheren/MQP_ws/MQP-Parachute-Packing-Automation-and-Inspection/src/top_cam_yolo/runs

DET=$BASE/detect/runs/detect/yolo26m_holes_all_aug/weights/best.pt
CLS=$BASE/classify/runs/classify/yolo26m_cls_custom_aug/weights/best.pt

source /home/zheren/MQP_ws/install/setup.bash

ros2 launch top_cam_loop_state top_cam_loop_state.launch.py \
    det_weights:=$DET \
    cls_weights:=$CLS \
    camera_index:=/dev/video0
