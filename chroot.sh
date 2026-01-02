#!/bin/sh 
export CHROOT=/home/pedrocasf/armv7_chroot
mount -t proc proc $CHROOT/proc/
mount -t sysfs sys $CHROOT/sys/
mount -o bind /dev/ $CHROOT/dev/
mount -o bind /dev/pts/ $CHROOT/dev/pts/
mount -o bind /run $CHROOT/run/
chroot $CHROOT/ sh -l
