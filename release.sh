#!/bin/bash
#
JOBS=`grep -c processor /proc/cpuinfo`
let JOBS=${JOBS}*2
JOBS="-j${JOBS}"
RELEASE_DATE=`date +%Y%m%d`
COMMIT_ID=`git log --pretty=format:'%h' -n 1`
ARM=arm
BOOT_PATH="arch/${ARM}/boot"
IMAGE="zImage"
DZIMAGE="dzImage"
MODEL=${1}
REGION=${2}
TIZEN_MODEL=tizen_${MODEL}



if [ "${REGION}" != "" ]; then
	TIZEN_MODEL=${TIZEN_MODEL}_${REGION}
fi

if [ "${MODEL}" = "" ]; then
	echo "Warnning: failed to get machine id."
	echo "ex)./release.sh model_name region_name"
	echo "ex)--------------------------------------------------"
	echo "ex)./release.sh	vasta"
	echo "ex)./release.sh	kiranlte"
	echo "ex)./release.sh	grandprime"
	echo "ex)./release.sh	j5lte"
	echo "ex)./release.sh	z3lte cis"
	echo "ex)./release.sh	z3lte chn"
	echo "ex)./release.sh	millet"
	echo "ex)./release.sh	hive"
	echo "ex)./release.sh	m2m"
	echo "ex)./release.sh	kitt"
	echo "ex)./release.sh	tetra"
	exit
fi

make ARCH=${ARM} ${TIZEN_MODEL}_defconfig
if [ "$?" != "0" ]; then
	echo "Failed to make defconfig :"$ARCH
	exit 1
fi

make ${JOBS} ARCH=${ARM} ${IMAGE}
if [ "$?" != "0" ]; then
	echo "Failed to make "${IMAGE}
	exit 1
fi

DTC_PATH="scripts/dtc/"

rm ${BOOT_PATH}/dts/*.dtb -f

make ARCH=${ARM} dtbs
if [ "$?" != "0" ]; then
	echo "Failed to make dtbs"
	exit 1
fi

./dtbtool -o ${BOOT_PATH}/merged-dtb -p ${DTC_PATH} -v ${BOOT_PATH}/dts/
if [ "$?" != "0" ]; then
	echo "Failed to make merged-dtb"
	exit 1
fi

mkdzimage -o ${BOOT_PATH}/${DZIMAGE} -k ${BOOT_PATH}/${IMAGE} -d ${BOOT_PATH}/merged-dtb
if [ "$?" != "0" ]; then
	echo "Failed to make mkdzImage"
	exit 1
fi

sudo ls > /dev/null
./scripts/mkmodimg.sh

if [ "${VARIANT}" != "" ]; then
	RELEASE_IMAGE=System_${TIZEN_MODEL}_${VARIANT}_${RELEASE_DATE}-${COMMIT_ID}.tar
else
	RELEASE_IMAGE=System_${TIZEN_MODEL}_${RELEASE_DATE}-${COMMIT_ID}.tar
fi

tar cf ${RELEASE_IMAGE} -C ${BOOT_PATH} ${DZIMAGE}
if [ "$?" != "0" ]; then
	echo "Failed to tar ${DZIMAGE}"
	exit 1
fi

tar rf ${RELEASE_IMAGE} -C usr/tmp-mod modules.img
if [ "$?" != "0" ]; then
	echo "Failed to tar modules.img"
	exit 1
fi

echo ${RELEASE_IMAGE}
