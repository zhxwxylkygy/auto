#!/bin/zsh
# 运行 ROS 2 启动文件
cd /root/master/program/RM2025-autoaim 

TIMEOUT=5  # 设定超时时间为10秒
NODE_NAMES=("/mcu" "/camera")  # 列出所有需要监控的节点名称，注意是用空格分隔
USER="$(whoami)" #用户名
WORKING_DIR="/root/master/program/RM2025-autoaim" # 代码目录
LAUNCH_FILE="rm2025autoaim rm.launch.py" # launch 文件
OUTPUT_FILE="$WORKING_DIR/screen.output" # 终端输出记录文件

rmw="rmw_fastrtps_cpp" #RMW
export RMW_IMPLEMENTATION="$rmw" # RMW实现

export ROS_HOSTNAME=$(hostname)
export ROS_HOME=${ROS_HOME:=$HOME_DIR/.ros}
export ROS_LOG_DIR="/tmp"

. /opt/ros/humble/setup.zsh
. $WORKING_DIR/install/setup.zsh

rmw_config=""
if [[ "$rmw" == "rmw_fastrtps_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export FASTRTPS_DEFAULT_PROFILES_FILE=$rmw_config
  fi
elif [[ "$rmw" == "rmw_cyclonedds_cpp" ]]
then
  if [[ ! -z $rmw_config ]]
  then
    export CYCLONEDDS_URI=$rmw_config
  fi
fi

function bringup() { 
    echo "111"
    source /opt/ros/humble/setup.zsh
    source $WORKING_DIR/install/setup.zsh
    ros2 launch rm2025autoaim  rm.launch.py > "$OUTPUT_FILE" 2>&1 
}

function record() {
   cd ./record
   ros2 bag record /camera/image /mcu/imu_msg /feature /tracker/marker /autoaimresult -b 3221225472 > /dev/null 2>&1 &
   cd ..
}

function restart() {
    ros2 daemon stop
    ros2 daemon start
    bringup
}

bringup

#record

# echo "launched"

# sleep $TIMEOUT

# while true; do
#     for node in "${NODE_NAMES[@]}"; do
#         topic="$node/heartbeat"
#         echo "- Check $node"
#         if ros2 topic list 2>/dev/null | grep -q $topic 2>/dev/null; then
#             data_value=$(timeout 8 ros2 topic echo $topic --once | awk '{print $1; exit}' 2>/dev/null)
#             if [ ! -z "$data_value" ]; then
#                 echo "    $node is OK! Heartbeat Count: $data_value"
#             else
#                 echo "    Heartbeat lost for $topic, restarting all nodes..."
#                 restart
#                 break 
            
#             fi
#         else
#             echo "    Heartbeat topic $topic does not exist, restarting all nodes..."
#             restart
#             break
#         fi
#     done
# done

