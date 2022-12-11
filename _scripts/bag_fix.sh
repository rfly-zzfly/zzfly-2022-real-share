#!/bin/bash 
SH_DIR=$(dirname $0)
BAG_DIR="${SH_DIR}/../../bag"

# file_name=$(basename $1)
file_name=$(basename $1 .bag.active)

pushd ${BAG_DIR}
echo ${file_name}

rosbag reindex "${file_name}.bag.active"
rosbag fix "${file_name}.bag.active" ${file_name}_fixed.bag
popd