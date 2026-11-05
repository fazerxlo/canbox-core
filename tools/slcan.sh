#!/usr/bin/env bash

set -eou pipefail

DEVICE=""

function help {
   echo "Configure slcan"
   echo
   echo "Syntax: slcan.sh [-d <device>|-k|-h|-v]"
   echo "options:"
   echo "d <dev> Specify ttyUSB device (e.g. /dev/ttyUSB0 or ttyUSB0). Defaults to QinHeng CH340."
   echo "h       Print this Help."
   echo "v       Verbose mode."
   echo "k       Close and kill slcan interface and daemon"
   echo ""
}

function slcan_kill {
    sudo ip link set dev can0 down || true
    sudo killall slcand  || true
}

function find_ch340_device {
    # 1. Search /dev/serial/by-id for QinHeng / CH340 / 1a86
    for dev in /dev/serial/by-id/*; do
        if [[ -e "${dev}" ]]; then
            if [[ "${dev}" =~ 1a86|CH340|ch340|ch341 ]]; then
                realpath "${dev}"
                return 0
            fi
        fi
    done

    # 2. Search /dev/ttyUSB* using udev properties
    for dev in /dev/ttyUSB*; do
        if [[ -e "${dev}" ]]; then
            if udevadm info -q property -n "${dev}" 2>/dev/null | grep -Eq 'ID_VENDOR_FROM_DATABASE=.*QinHeng|ID_MODEL_FROM_DATABASE=.*CH340|ID_VENDOR_ID=1a86|ID_MODEL_ID=7523'; then
                realpath "${dev}"
                return 0
            fi
        fi
    done

    # 3. Fallback to the first available ttyUSB device
    ls /dev/ttyUSB* 2>/dev/null | head -n 1 || true
}

function slcan_start {
    if [[ -n "${DEVICE}" ]]; then
        if [[ "${DEVICE}" == /* ]]; then
            usb_ttl_path="${DEVICE}"
        else
            #print available devices with number and read the user input to select proper device
            echo "Available devices:"
            ls /dev/ttyUSB* 2>/dev/null | nl -w2 -n | sed 's/^/  /'
            echo
            echo "Selected device: ${DEVICE}"
            usb_ttl_path="/dev/${DEVICE}"
        fi
    else
        usb_ttl_path=$(find_ch340_device)
        if [[ -n "${usb_ttl_path}" && -e "${usb_ttl_path}" ]]; then
            echo "Defaulting to QinHeng Electronics CH340 serial converter: ${usb_ttl_path}"
        fi
    fi

    if [[ -z "${usb_ttl_path}" || ! -e "${usb_ttl_path}" ]]; then
        echo "Error: Device '${DEVICE:-QinHeng CH340 / /dev/ttyUSB*}' not found" >&2
        exit 1
    fi

    usb_ttl_path=$(realpath "${usb_ttl_path}")

    usb_ttl=$(basename "${usb_ttl_path}")

    sudo slcan_attach -f -s4 -o "${usb_ttl_path}"
    sudo slcand -o -c -f -s4 -S 1000000 "${usb_ttl}" can0  
    sudo ip link set dev can0 up
}


while getopts ":d:hvk" option; do
   case $option in
      d)
         DEVICE="$OPTARG";;
      h) # display Help
         help
         exit;;
      v) # display Help
         set -x;;
      k) # display Help
         slcan_kill
         exit;;
      :) # Missing argument
         echo "Error: Option -$OPTARG requires an argument"
         exit 1;;
     \?) # Invalid option
         echo "Error: Invalid option"
         exit 1;;
   esac
done

slcan_kill
slcan_start

