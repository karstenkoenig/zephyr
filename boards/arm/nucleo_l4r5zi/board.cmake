board_runner_args(jlink "--device=stm32l4r5zi" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
