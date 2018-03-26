board_runner_args(jlink "--device=stm32l476rg" "--speed=4000")
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
