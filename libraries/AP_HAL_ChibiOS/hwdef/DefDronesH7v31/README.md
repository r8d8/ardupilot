# DefDrones H7 Basic V3.1 — DMA Map

Resolved 2026-09-01 by running ArduPilot's actual `dma_resolver.py` (via
`chibios_hwdef.py`) against [`hwdef.dat`](hwdef.dat),
from the `ardupilot` submodule (`r8d8/ardupilot`, `master`). MCU: STM32H743xx, DMAMUX-based
(DMA1 + DMA2, 8 streams each).

Note: `hwdef.dat` was moved here from clearwater `config/` onto Copter-4.7.0 with one API
rename applied (`HAL_PROBE_EXTERNAL_I2C_COMPASSES` → `define AP_COMPASS_PROBING_ENABLED 1`),
which the 4.7.0 hwdef parser requires. The rename does not change DMA allocation.
DMA stream assignment is fixed by the STM32H743 silicon/DMAMUX, not by firmware version, so
this map holds regardless of exact ArduPilot release flashed.

## UARTs

Every UART's RX gets its own dedicated (non-shared) DMA stream. All 7 UARTs' TX are
time-multiplexed onto one shared stream (safe via ChibiOS's shared-DMA locking, but only one
TX transfer runs at an instant).

**PCB silkscreen note:** labels like "RX5/TX5" refer to ArduPilot's logical `SERIALn` port
index, not the physical UART peripheral number — there is no physical UART5 used anywhere in
this hwdef (not declared, not in `SERIAL_ORDER`). The logical index is purely positional in
`SERIAL_ORDER OTG1 USART3 USART2 USART1 UART4 UART7 USART6 UART8 OTG2` (counting from 0), so
`SERIAL5` resolves to physical **UART7**. Confirmed independently by the relay comment in the
hwdef itself: `PB2 RELAY_7 ... #UART7 POWER RELAY +5VSERIAL5_EN`. That's the ELRS/RC-input port.

| UART | SERIALn | Role | RX DMA | TX DMA |
|---|---|---|---|---|
| USART3 | SERIAL1 | telem1 | DMA1 Stream 4 (dedicated) | DMA2 Stream 5 (shared) |
| USART2 | SERIAL2 | telem2 | DMA1 Stream 5 (dedicated) | DMA2 Stream 5 (shared) |
| USART1 | SERIAL3 | GPS | DMA1 Stream 6 (dedicated) | DMA2 Stream 5 (shared) |
| UART4 | SERIAL4 | GPS2 | DMA1 Stream 7 (dedicated) | DMA2 Stream 5 (shared) |
| UART7 | **SERIAL5** | **RC input (ELRS)** | DMA2 Stream 0 (dedicated) | DMA2 Stream 5 (shared) |
| USART6 | SERIAL6 | OSD telem | DMA2 Stream 1 (dedicated) | DMA2 Stream 5 (shared) |
| UART8 | SERIAL7 | ESC telem | DMA2 Stream 2 (dedicated) | DMA2 Stream 5 (shared) |

(`SERIAL0` = OTG1/USB, `SERIAL8` = OTG2/USB — no DMA relevance.)

The shared TX stream (`DMA2 Stream 5`) also carries `TIM2_UP`. This is fine in practice —
TX bursts on these ports are short/infrequent — and the latency-sensitive direction for RC
framing (UART7) and MAVLink/GPS parsing is fully dedicated on every port.

## SPI

| Bus | Device | RX DMA | TX DMA |
|---|---|---|---|
| SPI1 | ICM45686 (main IMU) | DMA1 Stream 0 (dedicated, `DMA_NOSHARE`) | DMA1 Stream 1 (dedicated, `DMA_NOSHARE`) |
| SPI2 | BMI270 + ICM45686_2 | DMA1 Stream 2 (dedicated, `DMA_NOSHARE`) | DMA1 Stream 3 (dedicated, `DMA_NOSHARE`) |
| SPI4 | MAX7456 (OSD) | DMA2 Stream 7 (shared w/ `TIM3_UP`) | DMA2 Stream 6 (shared w/ `TIM4_UP`) |

SPI1/SPI2 exclusivity is forced by `DMA_NOSHARE SPI1* SPI2* TIM1*` in the hwdef — this is what
keeps gyro sampling jitter-free. SPI4 (OSD) sharing is low-priority and not a concern.

## Timers / other

| Peripheral | DMA stream | Notes |
|---|---|---|
| TIM1 (UP, CH2, CH3) | DMA2 Stream 4 (dedicated, `DMA_NOSHARE`) | Bidirectional-DShot motor outputs (PE11/PE13/PE14) |
| ADC1 | DMA2 Stream 3 (dedicated) | Battery voltage/current sensing |
| TIM5_UP | **not resolved — no DMA** | Servo outputs on PA2/PA3 (PWM 11/12); plain PWM timer, not time-critical, no impact |

## How this was generated

```
python ardupilot/libraries/AP_HAL_ChibiOS/hwdef/scripts/chibios_hwdef.py \
  -D <scratch-outdir> \
  --params nonexistent.parm \
  <scratch-copy-of-hwdef.dat-with-compass-define-patched>
```

The `--params` flag works around an unrelated crash in this fork's `write_processed_defaults_file()`
when no defaults file is given (`os.path.join` on `None`). Output DMA map is in the generated
`hwdef.h`, under `// auto-generated DMA mapping from dma_resolver.py`.
