SRC_URI += "file://am335x-boneblack.dts;subdir=git/arch/boot/dts \
            file://am335x-boneblack01.dts;subdir=git/arch/${ARCH}/boot/dts \
            "

FILESEXTRAPATHS_prepend := "${THISDIR}/${PN}:"

PACKAGE_ARCH = "beaglebone"

KERNEL_DEVICETREE += "am335x-boneblack.dtb"

KERNEL_DEVICETREE += "am335x-boneblack01.dtb"

