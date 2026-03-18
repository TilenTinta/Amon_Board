#!/bin/sh

# Set custom board path
export ZEPHYR_BOARD_DIR=boards
echo $ZEPHYR_BOARD_DIR

export BOARD_ROOT=/workspace
echo $BOARD_ROOT

# Activate venv
#. source .venv/bin/activate

echo "Done!"
