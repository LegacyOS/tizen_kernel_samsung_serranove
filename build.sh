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

	make ${makeflags} -j4

	mv output/arch/arm/boot/zImage output/arch/arm/boot/boot.img-kernel

	./dtbtool -o output/arch/arm/boot/boot.img-dt -s 2048 -p output/scripts/dtc/ output/arch/arm/boot/dts/

}

build_kernel

#
