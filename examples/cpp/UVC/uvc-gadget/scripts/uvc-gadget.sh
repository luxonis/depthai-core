#!/bin/sh
# SPDX-License-Identifier: MIT

set -e

CONFIGFS="/sys/kernel/config"
GADGET="$CONFIGFS/usb_gadget"
VID="0x0525"
PID="0xa4a2"
SERIAL="0123456789"
MANUF=$(hostname)
PRODUCT="UVC Gadget"
BOARD=$(strings /proc/device-tree/model)
UDC=$(ls /sys/class/udc) # will identify the 'first' UDC

echo "Detecting platform:"
echo "  board : $BOARD"
echo "  udc   : $UDC"

create_frame() {
	# Example usage:
	# create_frame <function name> <width> <height> <format> <name>

	FUNCTION=$1
	WIDTH=$2
	HEIGHT=$3
	FORMAT=$4
	NAME=$5

	wdir=functions/$FUNCTION/streaming/$FORMAT/$NAME/${HEIGHT}p

	mkdir -p $wdir
	echo $WIDTH > $wdir/wWidth
	echo $HEIGHT > $wdir/wHeight
	echo $(( $WIDTH * $HEIGHT * 2 )) > $wdir/dwMaxVideoFrameBufferSize
	cat <<EOF > $wdir/dwFrameInterval
666666
100000
5000000
EOF
}

create_uvc() {
	# Example usage:
	#	create_uvc <target config> <function name>
	#	create_uvc config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "	Creating UVC gadget functionality : $FUNCTION"
	mkdir functions/$FUNCTION

	create_frame $FUNCTION 640 360 uncompressed u
	create_frame $FUNCTION 1280 720 uncompressed u
	create_frame $FUNCTION 320 180 uncompressed u
	create_frame $FUNCTION 1920 1080 mjpeg m
	create_frame $FUNCTION 640 480 mjpeg m
	create_frame $FUNCTION 640 360 mjpeg m

	mkdir functions/$FUNCTION/streaming/header/h
	cd functions/$FUNCTION/streaming/header/h
	ln -s ../../uncompressed/u
	ln -s ../../mjpeg/m
	cd ../../class/fs
	ln -s ../../header/h
	cd ../../class/hs
	ln -s ../../header/h
	cd ../../class/ss
	ln -s ../../header/h
	cd ../../../control
	mkdir header/h
	ln -s header/h class/fs
	ln -s header/h class/ss
	cd ../../../

	# Include an Extension Unit if the kernel supports that
	if [ -d functions/$FUNCTION/control/extensions ]; then
		mkdir functions/$FUNCTION/control/extensions/xu.0
		pushd functions/$FUNCTION/control/extensions/xu.0

		# Set the bUnitID of the Processing Unit as the XU's source
		echo 2 > baSourceID

		# Set this XU as the source for the default output terminal
		cat bUnitID > ../../terminal/output/default/bSourceID

		# Flag some arbitrary controls. This sets alternating bits of the
		# first byte of bmControls active.
		echo 0x55 > bmControls

		# Set the GUID
		echo -e -n "\x01\x02\x03\x04\x05\x06\x07\x08\x09\x0a\x0b\x0c\x0d\x0e\x0f\x10" > guidExtensionCode

		popd
	fi

	# Set the packet size: uvc gadget max size is 3k...
	echo 3072 > functions/$FUNCTION/streaming_maxpacket
	echo 2048 > functions/$FUNCTION/streaming_maxpacket
	echo 1024 > functions/$FUNCTION/streaming_maxpacket

	ln -s functions/$FUNCTION configs/c.1
}

delete_uvc() {
	# Example usage:
	#	delete_uvc <target config> <function name>
	#	delete_uvc config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "	Deleting UVC gadget functionality : $FUNCTION"
	rm $CONFIG/$FUNCTION

	rm functions/$FUNCTION/control/class/*/h
	rm functions/$FUNCTION/streaming/class/*/h
	rm functions/$FUNCTION/streaming/header/h/u
	rmdir functions/$FUNCTION/streaming/uncompressed/u/*/
	rmdir functions/$FUNCTION/streaming/uncompressed/u
	rm -rf functions/$FUNCTION/streaming/mjpeg/m/*/
	rm -rf functions/$FUNCTION/streaming/mjpeg/m
	rmdir functions/$FUNCTION/streaming/header/h
	rmdir functions/$FUNCTION/control/header/h
	rmdir functions/$FUNCTION
}

case "$1" in
    start)
	echo "Creating the USB gadget"

	echo "Creating gadget directory g1"
	mkdir -p $GADGET/g1

	cd $GADGET/g1
	if [ $? -ne 0 ]; then
	    echo "Error creating usb gadget in configfs"
	    exit 1;
	else
	    echo "OK"
	fi

	echo "Setting Vendor and Product ID's"
	echo $VID > idVendor
	echo $PID > idProduct
	echo "OK"

	echo "Setting English strings"
	mkdir -p strings/0x409
	echo $SERIAL > strings/0x409/serialnumber
	echo $MANUF > strings/0x409/manufacturer
	echo $PRODUCT > strings/0x409/product
	echo "OK"

	echo "Creating Config"
	mkdir configs/c.1
	mkdir configs/c.1/strings/0x409

	echo "Creating functions..."
	create_uvc configs/c.1 uvc.0
	echo "OK"

	echo "Binding USB Device Controller"
	echo $UDC > UDC
	echo "OK"
	;;

    stop)
	echo "Stopping the USB gadget"

	set +e # Ignore all errors here on a best effort

	cd $GADGET/g1

	if [ $? -ne 0 ]; then
	    echo "Error: no configfs gadget found"
	    exit 1;
	fi

	echo "Unbinding USB Device Controller"
	grep $UDC UDC && echo "" > UDC
	echo "OK"

	delete_uvc configs/c.1 uvc.0

	echo "Clearing English strings"
	rmdir strings/0x409
	echo "OK"

	echo "Cleaning up configuration"
	rmdir configs/c.1/strings/0x409
	rmdir configs/c.1
	echo "OK"

	echo "Removing gadget directory"
	cd $GADGET
	rmdir g1
	cd /
	echo "OK"
	;;
    *)
	echo "Usage : $0 {start|stop}"
esac
