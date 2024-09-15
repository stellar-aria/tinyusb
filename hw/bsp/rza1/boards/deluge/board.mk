# All source paths should be relative to the top level.
LD_FILE = ${BOARD_PATH}/deluge.ld


# For flash-jlink target
JLINK_DEVICE = R7S721020
JLINK_IF     = JTAG

# For flash-pyocd target
PYOCD_TARGET =

# flash using jlink
flash: flash-jlink
