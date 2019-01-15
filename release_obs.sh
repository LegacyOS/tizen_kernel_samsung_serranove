#!/bin/bash
#
JOBS=`grep -c processor /proc/cpuinfo`
let JOBS=${JOBS}*2
JOBS="-j${JOBS}"
ARM=arm
BOOT_PATH="arch/${ARM}/boot"
IMAGE="zImage"
DZIMAGE="dzImage"

CHIPSET=msm8916
RELEASE=${1}
MODEL=${2}
REGION=${3}
CARRIER=${4}
OPERATOR=${5}
VARIANT=""

if [ "${CARRIER}" != "" ]; then
	VARIANT="${MODEL}_${CARRIER}"
	if [ "${REGION}" != "" ]; then
		VARIANT="${VARIANT}_${REGION}"
		if [ "${OPERATOR}" != "" ]; then
			VARIANT="${VARIANT}_${OPERATOR}"
		else
			echo "Failed to get operator"
			exit 1
		fi
	else
		echo "Failed to get region"
		exit 1
	fi
else
	if [ "${REGION}" != "" ]; then
		MODEL="${MODEL}_${REGION}"
	fi
fi

if [ "${VARIANT}" = "" ]; then
	echo "base_def : ${MODEL}_defconfig , Release : ${RELEASE}"
else
	echo "base_def : ${MODEL}_defconfig , variant_def : ${VARIANT}_defconfig, Release : ${RELEASE}"
fi

if [ "${RELEASE}" = "usr" ]; then
	echo "Now disable CONFIG_SLP_KERNEL_ENG for ${MODEL}_defconfig"

	sed -i 's/CONFIG_SLP_KERNEL_ENG=y/\# CONFIG_SLP_KERNEL_ENG is not set/g' arch/${ARM}/configs/${MODEL}_defconfig
	if [ "$?" != "0" ]; then
		echo "Failed to disable CONFIG_SLP_KERNEL_ENG feature"
		exit 1
	fi
fi

if [ "${VARIANT}" = "" ]; then
	make ARCH=${ARM} ${MODEL}_defconfig
else
	make ARCH=${ARM} ${MODEL}_defconfig VARIANT_DEFCONFIG=${VARIANT}_defconfig
fi

if [ "$?" != "0" ]; then
	echo "Failed to make defconfig :"${ARCH}
	exit 1
fi

rm ${BOOT_PATH}/dts/*.dtb -f

make ${JOBS} ARCH=${ARM} ${IMAGE}
if [ "$?" != "0" ]; then
	echo "Failed to make "${IMAGE}
	exit 1
fi

make ARCH=${ARM} dtbs
if [ "$?" != "0" ]; then
	echo "Failed to make dtbs"
	exit 1
fi

DTC_PATH="scripts/dtc/"

dtbtool -o ${BOOT_PATH}/merged-dtb -p ${DTC_PATH} -v ${BOOT_PATH}/dts/
if [ "$?" != "0" ]; then
	echo "Failed to make merged-dtb"
	exit 1
fi

mkdzimage -o ${BOOT_PATH}/${DZIMAGE} -k ${BOOT_PATH}/${IMAGE} -d ${BOOT_PATH}/merged-dtb
if [ "$?" != "0" ]; then
	echo "Failed to make mkdzImage"
	exit 1
fi
