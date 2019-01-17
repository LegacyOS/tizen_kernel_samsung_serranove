output_path="./output"

export CROSS_COMPILE="/home/danil_e71/Tizen/toolchain/bin/arm-linux-gnueabi-"

export ARCH=arm
export TARGET_ARCH=arm

build_kernel()
{ 
	if [ ! -d "$output_path" ]; then
	   mkdir "$output_path"
	fi

	echo "${output_path}"

	makeflags+=" O=${output_path}"

	#make clean

	make ${makeflags}  msm8916_defconfig VARIANT_DEFCONFIG=tizen_serranovelte_defconfig

	make ${makeflags} -j4 zImage modules dtbs

}

build_kernel

#./dtbtool -o merged-dtb -p "scripts/dtc/" -v "arch/arm/boot/dts/"
