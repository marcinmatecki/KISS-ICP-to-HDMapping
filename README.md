# kiss-icp-converter

mkdir ros2_ws

cd ros2_ws

git clone https://github.com/marcinmatecki/kiss-icp-converter.git .

git submodule init

git submodule update --recursive

colcon build
