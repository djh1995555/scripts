#!/bin/bash
# =============================================================================
# revert_debug_script.sh - Revert debug modifications for parking controller
# =============================================================================
# This script restores the original files from backup copies
# =============================================================================

set -e  # Exit on error

# Get the directory where this script is located (project root)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "Project root: $SCRIPT_DIR"

# Define file paths
NODE_LAUNCH_CONF="$SCRIPT_DIR/mipilot/conf/cross_platform/parking/controller/node_launch_controller_debug.pb.conf"
PARKING_CONTROLLER_APP="$SCRIPT_DIR/mipilot/modules/parking/controller/app/parking_controller_processing_app.cc"
RUNNING_STATE="$SCRIPT_DIR/mipilot/modules/parking/controller/core/motion/apastate/running_state.cc"

# Define backup file paths
NODE_LAUNCH_CONF_BACKUP="${NODE_LAUNCH_CONF}.backup"
PARKING_CONTROLLER_APP_BACKUP="${PARKING_CONTROLLER_APP}.backup"
RUNNING_STATE_BACKUP="${RUNNING_STATE}.backup"

# Check if backup files exist
echo "Checking for backup files..."

MISSING_BACKUPS=0

if [ ! -f "$NODE_LAUNCH_CONF_BACKUP" ]; then
    echo "ERROR: Backup file not found: $NODE_LAUNCH_CONF_BACKUP"
    MISSING_BACKUPS=1
fi

if [ ! -f "$PARKING_CONTROLLER_APP_BACKUP" ]; then
    echo "ERROR: Backup file not found: $PARKING_CONTROLLER_APP_BACKUP"
    MISSING_BACKUPS=1
fi

if [ ! -f "$RUNNING_STATE_BACKUP" ]; then
    echo "ERROR: Backup file not found: $RUNNING_STATE_BACKUP"
    MISSING_BACKUPS=1
fi

if [ $MISSING_BACKUPS -eq 1 ]; then
    echo ""
    echo "ERROR: Some backup files are missing. Cannot revert."
    echo "Please ensure the apply_debug_script.sh was run first."
    exit 1
fi

echo "All backup files found."

# =============================================================================
# Restore files from backup
# =============================================================================
echo "Restoring files from backup..."

# Restore node_launch_controller_debug.pb.conf
cp "$NODE_LAUNCH_CONF_BACKUP" "$NODE_LAUNCH_CONF"
echo "Restored: $NODE_LAUNCH_CONF"
rm $NODE_LAUNCH_CONF_BACKUP

# Restore parking_controller_processing_app.cc
cp "$PARKING_CONTROLLER_APP_BACKUP" "$PARKING_CONTROLLER_APP"
echo "Restored: $PARKING_CONTROLLER_APP"
rm $PARKING_CONTROLLER_APP_BACKUP

# Restore running_state.cc
cp "$RUNNING_STATE_BACKUP" "$RUNNING_STATE"
echo "Restored: $RUNNING_STATE"
rm $RUNNING_STATE_BACKUP

echo ""
echo "=============================================="
echo "Debug modifications reverted successfully!"
echo "=============================================="
echo ""
echo "Restored files:"
echo "  1. $NODE_LAUNCH_CONF"
echo "  2. $PARKING_CONTROLLER_APP"
echo "  3. $RUNNING_STATE"
echo ""
echo "Backup files have been preserved."
echo "To apply debug modifications again, run: ./apply_debug_script.sh"
echo "=============================================="
