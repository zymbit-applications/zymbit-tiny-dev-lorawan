#!/bin/bash

project_home=~/zephyrprojects/applications/stm32wlfeather/stm32wl

echo Current Enivronment Variables:
env | grep ZEPHYR


# switch to the Zephyr source and set the environment variables
# cd ~/zephyrprojects_v3.3.0/zephyr_3.3.0/zephyr
source ~/zephyrprojects_v3.3.0/zephyr_3.3.0/zephyr/zephyr-env.sh

# Clear SDK cmake registration for cleanliness and rebuild
rm ~/.cmake/packages/Zephyr-sdk/*
# cd ~/zephyr-toolchains/zephyr-sdk-0.16.0
export ZEPHYR_SDK_INSTALL_DIR=~/zephyr-toolchains/zephyr-sdk-0.16.0
~/zephyr-toolchains/zephyr-sdk-0.16.0/setup.sh


echo New Enivronment Variables:
env | grep ZEPHYR

cd $project_home

rm -rf build/