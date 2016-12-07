#!/bin/sh

#echo -n 可以不换行
echo "Start to build ProjTest."

#设置一个目录用来存放编译的中间文件
#必须紧挨着赋值符号不能有空格
#pwd获取当前shell所在路径
#0是sh之后的第一个参数
DIR="$( cd "$( dirname "$0"  )" && pwd  )" #获取sh文件所在目录的脚本
build_dir="$DIR/build"
log_dir="$DIR/bin/main-log"
#echo $build_dir

#判断目录是否存在
#文档比较运算符  
#-e filename  假如 filename存在，则为真  [ -e /var/log/syslog ] 
#-d filename  假如 filename为目录，则为真  [ -d /tmp/mydir ] 
#-f filename  假如 filename为常规文档，则为真  [ -f /usr/bin/grep ] 
#-L filename  假如 filename为符号链接，则为真  [ -L /usr/bin/grep ] 
#-r filename  假如 filename可读，则为真  [ -r /var/log/syslog ] 
#-w filename  假如 filename可写，则为真  [ -w /var/mytmp.txt ] 
#-x filename  假如 filename可执行，则为真  [ -L /usr/bin/grep ] 
#注意[]中间必须加空格，否则不识别
if [ ! -d $build_dir ];then
mkdir $build_dir
fi

cd $build_dir

cmake .. -Wno-dev 

make -j8

cd ..

if [ ! -d $log_dir ];then
mkdir $log_dir
fi


echo "All done."
