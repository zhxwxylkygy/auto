# RM2025--AutoAim

PIE战队2025赛季自瞄代码

编译命令
 colcon build \
 --symlink-install \
 --event-handlers console_direct+ \
 --cmake-args \
 -DCMAKE_BUILD_TYPE=Release \
 -DCMAKE_CXX_FLAGS_RELEASE="-flto=16 -O3" \
 -DCMAKE_C_FLAGS_RELEASE="-flto=16 -O3" \
 -DCMAKE_CXX_FLAGS_DEBUG="-O0" \
 -DCMAKE_C_FLAGS_DEBUG="-O0" \
 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
 --parallel-workers 12
 
 
 
 #开机自起
 gnome-terminal -- bash -c "/bin/bash /home/wpie/program/RM2025-autoaim/autoaim-starter.sh;exec bash"
