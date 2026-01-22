#!/bin/bash

if ! command -v mcap >/dev/null 2>&1; then
    echo "Error: mcap is not installed"
    echo "Please refer https://mi.feishu.cn/wiki/V4v8wOOLUi5F5jkciXtcnu8wn4d to install mcap"
    exit 1
fi

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
    echo "Usage: $0 <record_file> [output_file]"
    exit 1
fi

RECORD_FILE="$1"
OUTPUT_DIR="${2:-/dat/prod/device_hub/}"
ATTACHMENT_NAME="/dat/prod/device_hub/system.json"
CHANNEL_NAME="/mcu/soc/carconfig_params"


if [[ "${OUTPUT_DIR:0:1}" != "/" ]]; then
	OUTPUT_FILE="(pwd)/{OUTPUT_FILE}"
fi

if [ ! -d "$OUTPUT_DIR" ]; then
	echo "Error: Directory $OUTPUT_DIR does not exist" >&2
	exit 1
fi

OUTPUT_FILE="$OUTPUT_DIR/system.json"
if [ -d "$OUTPUT_FILE" ]; then
	touch "$OUTPUT_FILE"
	echo "Warning: Create default output file system.json in $OUTPUT_DIR"
fi

if mcap list attachments "$RECORD_FILE" | grep -q "$ATTACHMENT_NAME"; then
    mcap get attachment "$RECORD_FILE" -n "$ATTACHMENT_NAME" -o "$OUTPUT_FILE"
    echo "Info: Extracted system.json from attachment"
else
    if ! mcap list channels "$RECORD_FILE" | grep -q "$CHANNEL_NAME"; then
        echo "Error: Channel $CHANNEL_NAME not found"
        exit 1
    fi
    
    TEMP_FILE=$(mktemp)
    mcap cat "$RECORD_FILE" --topics "$CHANNEL_NAME" --json > "$TEMP_FILE"
    
    if [ ! -s "$TEMP_FILE" ]; then
        echo "Error: No data found in channel $CHANNEL_NAME"
        rm -f "$TEMP_FILE"
        exit 1
    fi
    
    RANDOM_LINE=$(shuf -n 1 "$TEMP_FILE")
    
    echo "$RANDOM_LINE" | jq '{
        front_tyre_dimension: .data.frontTyreDimension,
        rear_tyre_dimension: .data.rearTyreDimension,
        rear_spoiler_type: .data.rearSpoilerType,
        eps_type: .data.epsType,
        vehicle_type: .data.vehicleType,
        vehicle_suit: .data.vehicleSuit,
        hardware_type: .data.hardwareType,
        streeing_ratio: .data.streeingRatio,
        damper_type: .data.damperType,
        spring_type: .data.springType,
        blind_spot_detection: .data.blindSpotDetection,
        rear_eds_type: .data.rearEdsType,
        flsr: .data.flsr,
        frsr: .data.frsr,
        rlsr: .data.rlsr,
        rrsr: .data.rrsr,
        scu: .data.scu
    }' > "$OUTPUT_FILE"

    rm -f "$TEMP_FILE"
    echo "Extracted car config from channel data"
fi
