# SPDX-License-Identifier: BSD-3-Clause

board_runner_args(jlink "--device=nrf52" "--speed=4000")
board_runner_args(nrfjprog "--softreset")
board_runner_args(pyocd "--target=nrf52840" "--frequency=4000000")
set (OPENOCD_NRF5_SUBFAMILY "nrf52")
include(${ZEPHYR_BASE}/boards/common/nrfjprog.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd-nrf5.board.cmake)
