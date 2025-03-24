export GZ_SIM_SYSTEM_PLUGIN_PATH=`pwd`/build

cd build
cmake ..
make

cd ..
xacro worlds/world.xacro > world.sdf

gz sim world.sdf
