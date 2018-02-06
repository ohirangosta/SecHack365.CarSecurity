#!/bin/bash

VID=""

if [ $# == 1 ]; then
	if [ "$1" == "alphard" ]; then
		VID="-DHA"
	elif [ "$1" == "corolla" ]; then
		VID="-DCF"
	elif [ "$1" == "aqua" ]; then
		VID="-DTA"
	elif [ "$1" == "debug" ]; then
		VID="-DTA"
	else
		echo "build type error"
		echo "Usage: $0 <alphard | corolla | aqua | debug>"
	fi
else
	echo "build type error"
	echo "Usage: $0 <alphard | corolla | aqua | debug>"
fi

if [ "$1" == "alphard" ]; then
	rm ./can2ip-transfer
	gcc -lm -pthread -lrt -o alphard2ip-transfer can_gps2ip-transfer.c gps_module.c "$VID"
elif [ "$1" == "corolla" ]; then
	rm ./can2ip-transfer
	gcc -lm -pthread -lrt -o carrolla2ip-transfer can_gps2ip-transfer.c gps_module.c "$VID"
elif [ "$1" == "aqua" ]; then
	rm ./can2ip-transfer
	gcc -lm -pthread -lrt -o aqua2ip-transfer can_gps2ip-transfer.c gps_module.c "$VID"
elif [ "$1" == "debug" ]; then
	rm ./can2ip-transfer.dbg
	gcc -lm -pthread -lrt -o can2ip-transfer.dbg can_gps2ip-transfer.c gps_module.c -DDEBAG "$VID"
fi
