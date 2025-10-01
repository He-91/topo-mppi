#!/bin/bash

# 清理A*相关文件的脚本
# 由于A*算法已经被MPPI完全替代，可以安全删除这些文件

echo "开始清理A*相关文件..."

# 检查文件是否存在再删除
if [ -f "/home/he/ros_ws/test/ego-planner/src/planner/path_searching/src/dyn_a_star.cpp" ]; then
    echo "删除 dyn_a_star.cpp"
    rm /home/he/ros_ws/test/ego-planner/src/planner/path_searching/src/dyn_a_star.cpp
fi

if [ -f "/home/he/ros_ws/test/ego-planner/src/planner/path_searching/include/path_searching/dyn_a_star.h" ]; then
    echo "删除 dyn_a_star.h"
    rm /home/he/ros_ws/test/ego-planner/src/planner/path_searching/include/path_searching/dyn_a_star.h
fi

echo "A*文件清理完成！"
echo ""
echo "现在可以重新编译项目："
echo "cd /home/developer/ros_ws/ego-planner"
echo "catkin_make"