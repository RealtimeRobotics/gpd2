#!/bin/bash

RED='\033[0;31m'
L_BLUE='\033[1;34m'
NC='\033[0m' # No Color
work_path="/home/luca/gpd_data"

gpd_pytorch_path="/home/luca/gpd_repo/gpd2/pytorch"

training_data_path="/home/luca/gpd_data/training_data"
num_channels=12

openVino_path="/opt/intel/computer_vision_sdk/deployment_tools/model_optimizer"

export_path="/home/luca/gpd_repo/gpd2/models/openvino/luca_models"


cd $work_path
echo -e "${L_BLUE}work path: "$work_path"${NC}"

echo -e "${L_BLUE}TRAINING...${NC}"

python $gpd_pytorch_path/train_net3.py $training_data_path/train.h5 $training_data_path/test.h5 $num_channels

echo echo -e "${L_BLUE}TRAINING: DONE${NC}"


echo -e "${L_BLUE}CONVERTING MODEL...${NC}"

python $gpd_pytorch_path/torch_to_onnx.py model.pwf model.onnx $num_channels

python3 $openVino_path/mo.py --input_model $work_path/model.onnx

echo -e "${L_BLUE}MODEL: GENERATED${NC}"


echo -e "${L_BLUE}EXPORTING...${NC}"

cp model.bin $export_path/model.bin
cp model.xml $export_path/model.xml

echo -e "${L_BLUE}DONE${NC}"
