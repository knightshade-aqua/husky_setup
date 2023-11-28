#!/bin/bash

## create ext4 partition
dev='/dev/nvme0n1'
sudo umount "$dev"
printf "o\nn\np\n1\n\n\nw\n" | sudo fdisk "$dev"
sudo mkfs.ext4 -F "${dev}p1"

## Change root to nvme
SD_DRIVE="${dev}p1"
sudo mount $SD_DRIVE /mnt

# Copy over the rootfs from the EMMC flash to the SDmmc
sudo rsync -axHAWX --numeric-ids --info=progress2 --exclude={"/dev/","/proc/","/sys/","/tmp/","/run/","/mnt/","/media/*","/lost+found"} / /mnt
# We want to keep the SDmmc mounted for further operations
# So we do not unmount the SDmmc
sync
echo 'The rootfs have copied to SDmmc.'

# Change the root parameter in extlinux.conf file
echo -n "Before extlinux.conf: " 
cat /boot/extlinux/extlinux.conf | grep "root="
echo $SD_DRIVE > .sd_drive_path.txt
sed -i 's/\//\\\//g' .sd_drive_path.txt
sudo sed -i 's/root=\/dev\/mmcblk0p1/root='$(cat .sd_drive_path.txt)'/g' /boot/extlinux/extlinux.conf
rm .sd_drive_path.txt
echo -n "After extlinux.conf:  " 
cat /boot/extlinux/extlinux.conf | grep "root="
echo 'extlinux.conf file updated.'