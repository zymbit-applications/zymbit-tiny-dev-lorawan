#!/bin/bash

project_home=~/zephyrprojects/applications/stm32wlfeather/stm32wl

echo Current Enivronment Variables:
env | grep ZEPHYR

# switch to the Zephyr source and set the environemtn variables
source ~/zephyrprojects/zephyr/zephyr-env.sh

# cd ~/zephyrprojects/zephyr
# source ./zephyr-env.sh

# Clear SDK cmake registration for cleanliness and rebuild
rm ~/.cmake/packages/Zephyr-sdk/*
# cd ~/zephyr-toolchains/zephyr-sdk-0.16.8
# export ZEPHYR_SDK_INSTALL_DIR=`pwd`
# ./setup.sh
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-toolchains/zephyr-sdk-0.16.8
~/zephyr-toolchains/zephyr-sdk-0.16.8/setup.sh

echo New Enivronment Variables:
env | grep ZEPHYR

cd $project_home

rm -rf build/