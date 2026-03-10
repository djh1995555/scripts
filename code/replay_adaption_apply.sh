#!/bin/bash
# =============================================================================
# apply_debug_script.sh - Apply debug modifications for parking controller
# =============================================================================
# This script performs the following modifications:
# 1. Comment out 3 lines in node_launch_controller_debug.pb.conf related to "/parking/avp/state_machine"
# 2. Add new input_channels with "/parking/router/status" before first output_channels
# 3. Update config file paths to absolute paths
# 4. Change false to true in CheckTimeValid and CheckObstacleMapTimeValid functions
# 5. Comment out "!parking_controller->GetMonitorSafetyFlag() ||" in running_state.cc
# =============================================================================

set -e  # Exit on error

# Get the directory where this script is located (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Project root: $SCRIPT_DIR"

# Define file paths
NODE_LAUNCH_CONF="$SCRIPT_DIR/mipilot/conf/cross_platform/parking/controller/node_launch_controller_debug.pb.conf"
PARKING_CONTROLLER_APP="$SCRIPT_DIR/mipilot/modules/parking/controller/app/parking_controller_processing_app.cc"
RUNNING_STATE="$SCRIPT_DIR/mipilot/modules/parking/controller/core/motion/apastate/running_state.cc"

# Verify files exist
echo "Verifying files exist..."
for file in "$NODE_LAUNCH_CONF" "$PARKING_CONTROLLER_APP" "$RUNNING_STATE"; do
    if [ ! -f "$file" ]; then
        echo "ERROR: File not found: $file"
        exit 1
    fi
done
echo "All files verified."

# =============================================================================
# Step 1 & 2: Modify node_launch_controller_debug.pb.conf
# - Comment out 3 lines with "/parking/avp/state_machine"
# - Add new input_channels with "/parking/router/status" before first output_channels
# =============================================================================
echo "Step 1 & 2: Modifying node_launch_controller_debug.pb.conf..."

# Backup original file (only if backup doesn't exist)
if [ ! -f "${NODE_LAUNCH_CONF}.backup" ]; then
    cp "$NODE_LAUNCH_CONF" "${NODE_LAUNCH_CONF}.backup"
fi

# Comment out the entire input_channels block containing "/parking/avp/state_machine"
# Original:
#       input_channels{
#         name : "/parking/avp/state_machine"
#       }
# Should become:
#       # input_channels{
#       #   name : "/parking/avp/state_machine"
#       # }
awk '
{
    line = $0
    if (line ~ /^[[:space:]]*input_channels[[:space:]]*\{/ && !in_block) {
        block_start = line
        in_block = 1
        next
    }
    if (in_block) {
        block_start = block_start "\n" line
        if (line ~ /^[[:space:]]*\}/) {
            # End of block, check if it contains /parking/avp/state_machine
            if (block_start ~ /\/parking\/avp\/state_machine/) {
                # Comment out each line
                n = split(block_start, lines, "\n")
                for (i = 1; i <= n; i++) {
                    if (lines[i] != "") {
                        print "      # " lines[i]
                    }
                }
            } else {
                print block_start
            }
            in_block = 0
            block_start = ""
            next
        }
        next
    }
    print line
}
' "$NODE_LAUNCH_CONF" > "${NODE_LAUNCH_CONF}.tmp" && mv "${NODE_LAUNCH_CONF}.tmp" "$NODE_LAUNCH_CONF"

# Add new input_channels before the first output_channels only
# Use awk for more reliable multi-line insertion
awk '
/^[[:space:]]*output_channels/ && !inserted {
    print "      input_channels {"
    print "        name : \"/parking/router/status\""
    print "      }"
    inserted = 1
}
{print}
' "$NODE_LAUNCH_CONF" > "${NODE_LAUNCH_CONF}.tmp" && mv "${NODE_LAUNCH_CONF}.tmp" "$NODE_LAUNCH_CONF"

echo "Step 1 & 2 completed."

# =============================================================================
# Step 3: Update config file paths to absolute paths
# =============================================================================
echo "Step 3: Updating config file paths..."

# Update ap_control_cmd.pb.conf path
sed -i "s|mipilot/conf/cross_platform/parking/controller/ap_control_cmd.pb.conf|${SCRIPT_DIR}/mipilot/modules/parking/controller/conf/bhd/ap_control_cmd.pb.conf|g" "$NODE_LAUNCH_CONF"

# Update parking_controller_config.json path
sed -i "s|mipilot/conf/cross_platform/parking/controller/parking_controller_config.json|${SCRIPT_DIR}/mipilot/conf/cross_platform/parking/controller/parking_controller_config.json|g" "$NODE_LAUNCH_CONF"

# Update vehicle_info.pb.conf path
sed -i "s|mipilot/eng/parking/offline_controller/conf/vehicle_info.pb.conf|${SCRIPT_DIR}/mipilot/conf/sensor_data/calib/vehicle/vehicle_info.pb.conf|g" "$NODE_LAUNCH_CONF"

echo "Step 3 completed."

# =============================================================================
# Step 4: Modify parking_controller_processing_app.cc
# - Change false to true in CheckTimeValid and CheckObstacleMapTimeValid functions
# =============================================================================
echo "Step 4: Modifying parking_controller_processing_app.cc..."

# Backup original file (only if backup doesn't exist)
if [ ! -f "${PARKING_CONTROLLER_APP}.backup" ]; then
    cp "$PARKING_CONTROLLER_APP" "${PARKING_CONTROLLER_APP}.backup"
fi

# In CheckTimeValid function, change "valid = false;" to "valid = true;"
# We need to be careful to only change it within the CheckTimeValid function
# The function has "valid = false;" inside an if block when time gap is exceeded

# Let's find the CheckTimeValid function and modify it
# The pattern is: if (dt_message > time_gap_ms) { valid = false; }
sed -i '/bool ControlProcessingApp::CheckTimeValid/,/^}$/ s/valid = false;/valid = true;/g' "$PARKING_CONTROLLER_APP"

# Similarly for CheckObstacleMapTimeValid - change "valid = false;" to "valid = true;"
sed -i '/bool ControlProcessingApp::CheckObstacleMapTimeValid/,/^}$/ s/valid = false;/valid = true;/g' "$PARKING_CONTROLLER_APP"

echo "Step 4 completed."

# =============================================================================
# Step 5: Modify running_state.cc
# - Comment out "!parking_controller->GetMonitorSafetyFlag() ||"
# =============================================================================
echo "Step 5: Modifying running_state.cc..."

# Backup original file (only if backup doesn't exist)
if [ ! -f "${RUNNING_STATE}.backup" ]; then
    cp "$RUNNING_STATE" "${RUNNING_STATE}.backup"
fi

# Comment out the line containing GetMonitorSafetyFlag in running_state.cc
# The pattern is: !parking_controller->GetMonitorSafetyFlag() ||
# Only add comment if not already commented (idempotent)
# Use awk to check if line is not already commented (//) before adding //
awk '
/!parking_controller->GetMonitorSafetyFlag\(\) *\|/ && $0 !~ /\/\// {
    sub(/!parking_controller->GetMonitorSafetyFlag\(\) *\|/, "// !parking_controller->GetMonitorSafetyFlag() ||")
}
{print}
' "$RUNNING_STATE" > "${RUNNING_STATE}.tmp" && mv "${RUNNING_STATE}.tmp" "$RUNNING_STATE"

echo "Step 5 completed."

# =============================================================================
# Summary
# =============================================================================
echo ""
echo "=============================================="
echo "Debug modifications applied successfully!"
echo "=============================================="
echo ""
echo "Modified files:"
echo "  1. $NODE_LAUNCH_CONF"
echo "  2. $PARKING_CONTROLLER_APP"
echo "  3. $RUNNING_STATE"
echo ""
echo ""
echo "To revert changes, run: ./revert_debug_script.sh"
echo "=============================================="
