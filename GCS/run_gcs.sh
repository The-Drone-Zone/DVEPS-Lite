#!/bin/bash

# Get the directory of the script
SCRIPT_DIR=$(dirname "$(realpath "$0")")

# Run the Python file from the same directory as the script
python3 "$SCRIPT_DIR/GCS.py"
