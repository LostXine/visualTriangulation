#!/bin/sh
echo "Start to clean ProjTest."

DIR="$( cd "$( dirname "$0"  )" && pwd  )" #获取sh文件所在目录的脚本
build_dir="$DIR/build"

#echo $build_dir

if [ -d "$build_dir" ];then
cd "$build_dir"
make clean
cd ..
rm -rf "$build_dir"
fi
#递归且不询问地删除整个文件夹
rm -rf bin
rm -rf lib

echo "All done."
