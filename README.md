# husky_setup
This repository contains instructions to setup clearpath husky with Ubuntu 20.04

# Ubuntu 20.04 installation on Clearpath Husky
*Download SDK from: https://developer.nvidia.com/sdk-manager on the host PC.
*Connect the host PC to the Jetson board. (The board can be ON or OFF)
*Start the SDK manager on the host PC.
*The jetson device should automatically detected. Your host PC OS should also be automatically detected.
*Select the required jetpack version.
*You can select the components to be downloaded in the Step 2.
*Continue with Step 3 to install the components. NOTE: Keep the storage device as EMMC. If you change it to NVME the flash will not be sucessful. It will give errors when trying to boot the Nvidia jetson.
*After the flash process is complete, do not shut  the installation process, but do not continue with it either, instead connect the zed box to a monitor and keyboard.
*Run the script file. This will change the memory from emmc to main memory.
*Once the script has completed its procedure, reboot the jetson and continue with the installation process.
