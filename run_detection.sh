#!/bin/sh
echo "Start to clean ProjTest."

DIR="$( cd "$( dirname "$0"  )" && pwd  )" #获取sh文件所在目录的脚本
build_dir="$DIR/bin"

#echo $build_dir

${build_dir}/detection ~/Codelibs/ssd/caffe/models/deploy.prototxt ~/Codelibs/ssd/caffe/models/VGG_SJTUBMW-B_SSD_1280x720_iter_60000.caffemodel /home/xine/Tri2/s1/

