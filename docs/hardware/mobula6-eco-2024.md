# HDZero Mobula6 ECO 2024: hardware reference for ironloop

Target airframe: **HDZero / Happymodel Mobula6 ECO 2024** (HDZero SKU HDZ3661, Happymodel HM-5A10090).
1S 65 mm brushless whoop with the **Happymodel SuperX HD ELRS** AIO flight controller (board rev V1.1 in photos),
the **HDZero ECO VTX** and the **HDZero ECO camera**.

Confidence tags: **[src]** comes from a primary source listed at the bottom. **[derived]** is computed from sourced values.
**[est]** is an engineering estimate that needs measuring on the real quad. **[verify]** means sources disagree or are silent.

## Summary

| Item | Value |
|---|---|
| FC board | Happymodel SuperX HD ELRS 1-2S AIO; Betaflight target **CRAZYBEEF4DX** (manufacturer `HAMO`) [src] |
| MCU | **STM32F411CEU6** (UFQFPN48), Cortex-M4F; 512 KB flash, 128 KB SRAM; 100 MHz max [src] |
| Crystal | HSE **8 MHz** (`SYSTEM_HSE_MHZ 8`) [src] |
| IMU | **BMI270 _or_ ICM-42688-P** depending on batch, on **SPI1**; firmware must probe both [src] |
| Blackbox flash | SPI NOR on **SPI2** (target enables W25Q128FV driver; capacity on this board not stated) [src][verify] |
| Analog OSD | none (MAX7456 not populated) [src] |
| ESC | onboard 4-in-1, 12 A cont. / 15 A 3 s, EFM8BB21 [derived from layout name], **Bluejay 0.19, layout Z_H_30, 48 kHz** [src] |
| Motor protocol | **DShot300, bidirectional ON** (factory), 12 motor poles [src] |
| Receiver | onboard **ESP8285 ExpressLRS 2.4 GHz** (same as Happymodel EP1), **CRSF on USART2**, ELRS 3.0.1 [src] |
| VTX | **HDZero ECO**, **MSP + DisplayPort on USART1** at 115200 baud, powered from the FC's +5 V BEC [src] |
| Battery sense | VBAT ADC on **PB0**, current ADC on **PB1** [src] |
| BEC | 5 V, 1.5 A max [src] |
| Bootloader | STM32 ROM **USB DFU** (PA11/PA12); "F411 BOOT" pad on the board [src] |
| Airframe | 65 mm wheelbase, **23.16 g without battery**, SE0702 **28000 KV** motors, Gemfan **1208 31 mm tri-blade** props [src] |

## MCU and clocks

- STM32F411CE: flash at 0x0800_0000. Sectors are 4×16 KB, then 1×64 KB, then 3×128 KB (metapac). SRAM is 128 KB at 0x2000_0000 [src].
- Betaflight runs this board at `cpu_overclock = 108MHZ` (VCO 432 MHz, P/4 = 108 MHz, Q/9 = 48 MHz) [src].
- A 100 MHz SYSCLK cannot also give an exact 48 MHz USB clock from the PLL. Suggested in-spec config with USB [derived]:
  HSE 8 MHz → PLLM /4 (2 MHz) → ×192 (VCO 384 MHz) → **PLLP /4 = 96 MHz SYSCLK**, **PLLQ /8 = 48 MHz USB**.
  AHB /1, **APB1 /2 (48 MHz; TIM2-5 at 96 MHz)**, APB2 /1 (96 MHz; TIM1/9/10/11 at 96 MHz).
- No SWD in practice: **PA14 (SWCLK) is the flash chip's CS**, and SWD pads are not broken out in any photo. Plan on USB DFU for flashing and USB CDC for logs (no RTT/probe-rs). [src][derived]

## Pin map (Betaflight unified target + Happymodel factory dump)

| Function | Pin | Peripheral / notes |
|---|---|---|
| Motor 1 | **PB10** | TIM2_CH3, AF1 |
| Motor 2 | **PB6** | TIM4_CH1, AF2 |
| Motor 3 | **PB7** | TIM4_CH2, AF2 |
| Motor 4 | **PB8** | TIM4_CH3, AF2 |
| Gyro SCK / MISO / MOSI | PA5 / PA6 / PA7 | SPI1 (AF5) |
| Gyro CS | **PA4** | GPIO |
| Gyro INT (EXTI) | **PA1** | data-ready interrupt |
| Flash SCK / MISO / MOSI | PB13 / PB14 / PB15 | SPI2 (AF5) |
| Flash CS | **PA14** | also SWCLK |
| (OSD CS, unpopulated) | PB12 | SPI2 |
| SPI3 (unused) | PB3 / PB4 / PB5 | declared in target, nothing attached |
| USART1 TX / RX → **HDZero VTX** | **PA9 / PA10** | pads TX1 / RX1 (AF7) |
| USART2 TX / RX → **ELRS RX** | **PA2 / PA3** | internal; also on pads TX2 / RX2 (AF7) |
| VBAT ADC | **PB0** | ADC1_IN8 |
| Current ADC | **PB1** | ADC1_IN9 |
| Status LED (green, top) | **PC13** | active-low (BF drives it low for "on" with `led_inversion = 0`) [derived] |
| Beeper | **PC15** | BUZ+/BUZ− pads; `BEEPER_INVERTED` / `beeper_inversion = ON` → treat as active-high [verify] |
| LED strip | PA0 | TIM5_CH1, AF2; pad `LED_S` (nothing fitted) |
| USB D− / D+ | PA11 / PA12 | OTG_FS |

Pad notes: `IRX1(SBUS)` is an inverted input that shares RX1, so "IRX1 and RX1 cannot be used at the same time" [src].
The ESP8285 has its own BOOT pad for reflashing the receiver [src].

## IMU

- Two possible chips. Probe at boot and branch [src]:
  - **BMI270**: `CHIP_ID` reg 0x00 = **0x24**. SPI ≤ **10 MHz**. A dummy read is needed to switch it to SPI mode. It needs the ~8 KB Bosch config blob uploaded at init. Betaflight runs gyro ODR **3.2 kHz** and accel at 800 Hz.
  - **ICM-42688-P**: `WHO_AM_I` reg 0x75 = **0x47**. SPI ≤ **24 MHz**. Betaflight runs ODR 8 kHz (or 4/2/1 kHz) and picks an AAF cutoff of 258/536/997/1962 Hz.
- Data-ready on **PA1**. Betaflight uses `gyro_hardware_lpf = NORMAL` and `pid_process_denom = 2`, so the PID loop runs at 1.6 kHz (BMI270) or 4 kHz (ICM42688 at 8 kHz) [src].
- **Alignment [verify]:** the unified target says `GYRO_1_ALIGN CW90_DEG` (that is the original CrazyBee F4DX). Happymodel's factory dumps for *this* quad (BF 4.4.2 and 4.5.0) explicitly set `gyro_1_sensor_align = CW0` (not `DEFAULT`) with board alignment 0/0/0. Treat **CW0** as correct for the SuperX HD, and confirm with a tilt test on hardware before the first arm. No source gives a separate orientation per chip.

## Motors, ESC, DShot

- Factory Betaflight: `motor_pwm_protocol = DSHOT300`, `dshot_bidir = ON`, `dshot_edt = OFF`, `motor_poles = 12`, `dshot_burst = AUTO`, `dshot_bitbang = OFF`, `dshot_idle_value = 1000` (10.00 % on BF 4.5; 600 = 6 % on 4.4.2), `yaw_motors_reversed = ON` (**props out**) [src].
- ESC: Bluejay **0.19**, `Z_H_30` (48 kHz PWM; "H" is EFM8BB21 in BLHeli_S/Bluejay layout naming [derived]). The manual asks for **Startup power min 1100 / max 1200**. It also shows Temperature protection 140 °C, timing 22.5°, Demag Low, and Rampup 9x [src]. Bluejay supports bidirectional DShot (eRPM telemetry) and DShot commands such as beacons and direction [src]. Extended DShot Telemetry arrived after 0.19 [verify], so expect eRPM only.
- DShot300 timing: bit period 3.333 µs, '1' high 2.5 µs, '0' high 1.25 µs, 16-bit frame ≈ 53 µs [derived]. Bidirectional mode inverts the line (idle high, CRC inverted). About 30 µs after the frame the ESC answers on the same wire with a 21-bit GCR eRPM frame at 5/4 × bit rate (375 kbit/s). Mechanical RPM = eRPM / **6** (12 poles → 6 pole pairs).
- Crate: `dshot-frame` 0.4.0 has normal and bidirectional frame encoding plus `ErpmTelemetry` decoding (needs the GCR-decoded 16-bit value).

### Motor layout (Betaflight QuadX, viewed from top, front = ELRS antenna / camera side)

| Motor | Position | Pin | Prop spin (props out) | Prop (manual) |
|---|---|---|---|---|
| M1 | rear-right | PB10 | CCW | 1208 |
| M2 | front-right | PB6 | CW | 1208R |
| M3 | rear-left | PB7 | CW | 1208R |
| M4 | front-left | PB8 | CCW | 1208 |

The board photos agree: top view has M4 upper-left, M2 upper-right, M3 lower-left, M1 lower-right, with USB at the rear [src].
Spin directions follow from BF's "props in" defaults (M1 CW, M2 CCW, M3 CCW, M4 CW) reversed by `yaw_motors_reversed = ON` [derived].
Diagonal pairs M1+M4 and M2+M3 share a prop type, which matches the manual. Verify the spin on the bench with DShot beeps or a slow spin.

### DMA plan for embassy (F411 request map from stm32-metapac)

- TIM2: CH3 → DMA1 S1 ch3, UP → DMA1 S1/S7 ch3.
- TIM4: CH1 → DMA1 S0, CH2 → DMA1 S3, CH3 → DMA1 S7, **UP → DMA1 S6** (all ch2).
- Conflicts: TIM4_UP (S6) collides with USART2_TX. TIM4_CH2 (S3) collides with SPI2_RX. That is why Betaflight uses burst (`DMAR`) mode and runs the UARTs without DMA.
- **Recommendation for bidirectional DShot: GPIO bit-bang.** All four motor pins are on **GPIOB** (PB6/7/8/10), so one DMA stream writing `GPIOB->BSRR` drives all four motors at once. Another stream sampling `GPIOB->IDR` captures all four eRPM replies. On the F4, only **DMA2** can reach AHB1 GPIO, so pace it with **TIM1** (TIM1_UP = DMA2 S5 ch6; CH1-CH4 on DMA2 S1/S2/S3/S4/S6). This is how Betaflight's bit-bang driver works on F4 [derived]. embassy-stm32 has no DShot driver, so this needs a small custom driver on `embassy_stm32::pac` plus `dma::Transfer`.
- Output-only alternative: `SimplePwm::waveform_up_multi_channel` (TIM4 CH1..CH3 burst on DMA1 S6) plus `waveform`/`waveform_up` for TIM2_CH3 (DMA1 S1). Receiving bidirectional replies through timer input capture is fiddlier, because the channel must switch from output to input-capture DMA after every frame.
- Suggested allocation: DMA1 S1 TIM2, S6 TIM4_UP, S3/S4 SPI2 (flash), S5 USART2_RX (CRSF). DMA2 S0/S3 SPI1 RX/TX (IMU), S4 ADC1, S2 USART1_RX, S7 USART1_TX. Use S5/S1/S6 for TIM1 bit-bang if chosen.

## Receiver (ELRS)

- ESP8285 receiver wired to **USART2 (PA2 TX / PA3 RX)**, protocol **CRSF**, `serialrx_inverted = OFF`, `serialrx_halfduplex = OFF`, `crsf_use_negotiated_baud = OFF` → **420000 baud 8N1, full duplex** [src].
- CRSF RC frames: type 0x16, 16 channels × 11 bit, sync byte 0xC8, CRC8 poly 0xD5. Telemetry (battery 0x08, attitude 0x1E, flight mode 0x21, link stats 0x14 from the RX) goes back on PA2. Crate: `crsf` 2.0.1 (`no_std` parser).
- Factory channel map **AETR1234** (`map AETR1234`; the manual says TAER is also acceptable on the radio side) [src].
- Factory modes: arm = AUX1 1550-2100 µs; angle = AUX2 1300-1700; turtle (box 35) = AUX3 1600-2100. Failsafe is `DROP` after a 1.5 s guard [src].
- Binding happens on the ESP8285 by itself (power-cycle 3×, then the red LED double-flashes), so the FC firmware needs nothing for it [src]. ELRS firmware updates still work over WiFi or the ESP BOOT pad; Betaflight passthrough will not work unless ironloop implements it.
- Do not mount the RX on USART1: the ROM bootloader listens there (see Flashing). USART2's bootloader pins are PD5/PD6, which this package does not have, so CRSF traffic cannot hijack the ROM bootloader [src].

## HDZero ECO VTX (video, OSD, VTX control)

- Wiring (manual photo): VTX `T` → FC **RX1 (PA10)**, VTX `R` ← FC **TX1 (PA9)**, VTX power from FC **+5V**, GND [src]. VTX input range 4-12 V, consumption 5-6 W, RF 25/200 mW, no SmartAudio or Tramp [src].
- Video works with no FC link at all (the camera connects straight to the VTX). The UART is only for OSD, VTX channel/power control, and arm status.
- Protocol: **MSP v1 at 115200 8N1** (the VTX also tries 230400 and switches if it gets no valid OSD frames for about 2 s). The VTX is the MSP *master*: it starts transmitting after the first valid frame from the FC, or 3 s after boot. It then polls these, which the FC must answer [src, hdzero-vtx `msp_displayport.c`]:
  - `MSP_FC_VARIANT` (2): answer `"BTFL"` so it initialises the VTX table and the HD canvas (`"INAV"` is also understood).
  - `MSP_GET_VTX_CONFIG` (88), `MSP_STATUS` (101; arm flag), `MSP_RC` (105; stick menus while disarmed), `MSP_BOXIDS` (119), `MSP_GET_OSD_CANVAS` (0xBD = 189).
  - The VTX sends `MSP_SET_VTX_CONFIG` (89) and `MSP_SET_OSD_CANVAS` (188).
- OSD output: the FC sends `MSP_DISPLAYPORT` (182) commands with subcommands heartbeat 0, release 1, clear 2, write string 3 (row, col, attr, text), draw/commit 4. The canvas is **50×18 (HD)** or 30×16 (SD) [src].
- Factory VTX settings: R7 5880 MHz, power index 1 (25 mW). The table has powers 25/200 and is auto-filled by the VTX [src]. The HDZero `TEAM_RACE` option holds RF at 0 mW until the FC reports armed over MSP, so a broken MSP link can look like "no video" [src].
- Crate: `multiwii_serial_protocol_v2` 0.1.12 (2020, stale). A hand-written MSP v1 encoder and parser is about 100 lines.

## Battery voltage and current

- Battery: 1S LiPo/LiHV only on this quad (the FC accepts 2.8-8.7 V). The connector is **GNB A30** [src].
- **VBAT (PB0):** `vbat_scale = 110`, `vbat_divider = 10`, `vbat_multiplier = 1`. Betaflight computes `V = adc × Vref × 110 / (4095 × 10 × 100)`, which is a **10k:1k divider (×11)**. At Vref 3.3 V that is ≈ 8.86 mV/LSB, and full scale is 36.3 V [src][derived]. At 4.35 V the raw reading is about 491 counts, so oversample or filter.
- **Current (PB1):** `ibata_scale = 470`, `ibata_offset = 0` (the target default of 179 is overridden). Betaflight scale units are mV per 10 A, so **I = V_adc / 47 mV per A** with no offset [src][derived].
- An independent measurement found this exact quad's current sensor **reads about 25 % high**, and the OSD showed 4.15 V for a true 4.35 V at plug-in [src: QuadMath]. Calibrate both channels against a meter. ironloop measures the ADC reference against the factory VREFINT calibration and divides the current by 1.25.
- Factory limits: cell max 4.30 V, full 4.10 V, warning 3.20 V, min 3.10 V [src].
- ADC1 DMA: DMA2 S4 (or S0) ch0 [src].

## Flashing and restoring Betaflight

- **ROM DFU** (STM32 system memory at 0x1FFF_0000, bootloader ID 0xD0, USB VID:PID **0483:DF11**). It needs the 8 MHz HSE, which this board has [src].
- Ways in:
  1. Hold or bridge the **"F411 BOOT"** pad (next to the MCU, top side) to 3.3 V while plugging in USB [src].
  2. From firmware: write a magic value to RAM or a backup register and reset. Early in boot, before clock setup, check the magic, set MSP from `*(0x1FFF0000)` and jump to `*(0x1FFF0004)`. This is what Betaflight's `systemResetToBootloader(BOOTLOADER_REQUEST_ROM)` does; BF triggers it via CLI `bl`, MSP reboot mode 1, or the `R` character on an MSP port [src]. ironloop does this on a disarmed stick gesture (`firmware/mobula6/src/dfu.rs`).
- Flash with `dfu-util -a 0 -s 0x08000000:leave -D ironloop.bin`, STM32CubeProgrammer, or Betaflight Configurator ("Load Firmware [Local]" with a .hex). `probe-rs` cannot reach the chip without SWD.
- The ROM bootloader also listens on **USART1 (PA9/PA10)**, which is where the VTX is. If DFU fails to enumerate, unplug the battery so the VTX is unpowered, or disconnect the VTX connector [derived].
- **Restoring Betaflight:** enter DFU, flash target **CRAZYBEEF4DX** (BF 4.4.2 or 4.5.x) from Betaflight Configurator, then paste Happymodel's CLI dump for this quad. The dump sets the gyro alignment, current scale, ports, PIDs and VTX table. ESC (Bluejay) and ELRS firmware live on their own MCUs and are not touched by flashing the F411.

## Physics model parameters

| Parameter | Value | Tag |
|---|---|---|
| Mass without battery | 23.16 g (Happymodel scale photo; getfpv lists 23.3 g) | src |
| Typical battery | 1S 300 mAh LiHV ≈ 8.2 g; 450 mAh ≈ 11-12 g | src / est |
| AUW (300 mAh) | ≈ 31.4 g | derived |
| Wheelbase (motor to motor, diagonal) | 65 mm → arm 32.5 mm, motor x/y offset ±22.98 mm | src / derived |
| Size | 81 × 81 × 45 mm | src |
| Motor | Happymodel SE0702, **28000 KV**, 9N12P (6 pole pairs), 7 mm × 2 mm stator, 1 mm shaft, 1.49 g | src |
| Prop | Gemfan 1208 tri-blade, **31 mm** (1.22"), 0.8" pitch, 0.21 g, PC | src |
| Motor no-load | 2.0 V: 0.55 A, 55 960 rpm | src |

Bench thrust per motor, SE0702 28000 KV at 3.7 V with a **Gemfan 1219 31 mm tri-blade** (higher pitch than the stock 1208, so the stock prop gives somewhat less thrust and current) [src]:

| Current (A) | Thrust (g) | Power (W) | g/W |
|---|---|---|---|
| 1.03 | 7.6 | 3.81 | 1.99 |
| 2.05 | 14.0 | 7.59 | 1.85 |
| 3.07 | 19.5 | 11.36 | 1.72 |
| 4.03 | 23.0 | 14.91 | 1.54 |
| 4.74 | **24.6** (max) | 17.54 | 1.40 |

Derived and estimated values for the simulator:

- Max total thrust ≈ 4 × 24.6 g ≈ 98 g (≈ 0.97 N) → **thrust-to-weight ≈ 3.1** at 31.4 g AUW [derived]. Hover needs ≈ 7.9 g per motor, which is about 32 % of max thrust and ≈ 1.05 A per motor on the bench curve [derived].
- Measured flight: 300 mAh LiHV, AUW 27.7 g (Mobula6 2024 HDZero with SE0702 28k), **6.3 A average at cruise, 154 s** [src: QuadMath]. The HDZero ECO VTX alone draws 5-6 W, about 1.5 A from 1S through the BEC [derived].
- Thrust coefficient: assuming loaded full-throttle speed ≈ 0.7 × KV × 3.7 V ≈ 72 500 rpm (ω ≈ 7 590 rad/s), **k_T ≈ 4.2e-9 N/(rad/s)²**, which is C_T ≈ 0.15 in `T = C_T ρ n² D⁴` [est]. Fit the real value from bidirectional eRPM plus a thrust stand or hover logs.
- Drag torque: k_Q / k_T ≈ **0.0075 m** (from ≈ 80 % of 17.5 W shaft power at max) [est].
- Motor and prop spin-up time constant ≈ 20-30 ms [est].
- Inertia (point masses: 4 × 1.7 g motor+prop at r = 32.5 mm, 24 g core block 30 × 30 × 20 mm, about 3 g of ducts): **Ixx ≈ Iyy ≈ 1.0e-5 kg·m², Izz ≈ 1.6e-5 kg·m²** [est].
- Battery: 4.35 V full (LiHV), sags to about 3.3 V under load near the end [src]. Internal resistance is not published; ≈ 0.1-0.2 Ω is a reasonable starting point for 300 mAh whoop packs [est].
- Factory Betaflight tune, as a sanity reference: P/I/D/FF roll 68/60/62/239, pitch 71/63/67/249, yaw 68/60/0/239; dynamic idle off; RPM filter 3 harmonics with min 100 Hz [src].

## Embassy support

- **`embassy-stm32` 0.6.0** (2026-03-20) has feature **`stm32f411ce`**; use it with `stm32-metapac` 21, plus features `rt`, `memory-x`, `exti`, `time-driver-tim9`, and `defmt` if wanted [src].
  - `time-driver-any` would pick **TIM9** on this chip, which is free here (Betaflight only uses it for PPM/PWM input on the UART2 pins). Naming it explicitly is clearer. Keep TIM2/TIM4 for motors, TIM5 for the LED strip, and TIM1 for DShot bit-bang.
  - Clock config is `rcc::Config` with `hse: Some(Hse { freq: 8 MHz, mode: Oscillator })`, `pll_src: HSE`, `pll: Some(Pll { prediv: DIV4, mul: MUL192, divp: Some(DIV4), divq: Some(DIV8), .. })`, `sys: PLL1_P`, `apb1_pre: DIV2`.
  - Available drivers: SPI with DMA (async), USART (async, DMA or ring buffer), ADC, EXTI, `SimplePwm` with DMA waveforms (`waveform`, `waveform_up`, `waveform_up_multi_channel`), `InputCapture`, and USB OTG FS (`usb::Driver::new_fs`, PA12/PA11, with `embassy-usb` 0.6 for CDC-ACM).
- `embassy-executor` 0.10, `embassy-usb` 0.6, target `thumbv7em-none-eabihf`.
- embassy has none of the board-specific pieces; ironloop writes its own DShot bit-bang driver, BMI270 and ICM-42688 init, CRSF parser, and ROM-bootloader jump. MSP DisplayPort is still to do.

## Open items to check on the real quad

1. Which IMU is fitted (read the chip ID), and the gyro orientation (CW0 per factory dump vs CW90 per target), on a BMI270 batch and on an ICM-42688 batch.
2. Flash chip JEDEC ID and size.
3. Status LED polarity on PC13 and beeper polarity on PC15.
4. Motor spin directions and Bluejay settings (EDT availability).
5. VBAT and current scale calibration.
6. Measured thrust and eRPM curve with the stock 1208 props.

## Sources

- Happymodel SuperX HD ELRS product page and pad diagrams: <https://www.happymodel.cn/index.php/2023/09/04/superx-hd-elrs-1-2s-aio-flight-controlelr-built-in-12a-esc-and-uart-elrs-receiver-for-digital-whoop/>
- Mobula6 ECO 2024 manual (PDF): <https://www.happymodel.cn/wp-content/uploads/2024/01/Mobula6-ECO-2024-1s-micro-FPV-Whoop-manual.pdf>
- Happymodel factory CLI dumps (BF 4.4.2 and 4.5.0): <https://www.happymodel.cn/index.php/2024/03/11/dump-cli-file-for-mobula6-eco-2024/>
- HDZero product page: <https://www.hd-zero.com/product-page/hdzero-mobula6-eco-2024>; retailer specs: Unmanned Tech, GetFPV
- Betaflight unified target: `betaflight/config` `configs/HAMO/CRAZYBEEF4DX/config.h` (master, 2026-11-15)
- Betaflight source (`sensors/voltage.c`, `sensors/current.c`, `drivers/accgyro/*`, `flight/mixer_init.c`, `drivers/light_led.c`, `msp/msp_serial.c`, `cli/settings.c`)
- HDZero VTX firmware: <https://github.com/hd-zero/hdzero-vtx> (`src/msp_displayport.c`, `src/msp_protocol.h`); HDZero VTX docs <https://hd-zero.github.io/hdzero-vtx-docs/latest/>
- SE0702 bench data: <https://www.happymodel.cn/index.php/2021/08/20/happymodel-se0702-kv26000-brushless-motors-for-1s-fpv-brushless-whoop>
- QuadMath measured flights: <https://quadmath.com/bench.html>
- ST AN2606 (STM32F411 system-memory bootloader, Rev 70/71)
- crates.io: embassy-stm32 0.6.0 (source and `stm32-metapac` 21 F411CE metadata inspected), dshot-frame 0.4.0, icm426xx 0.4.0, bmi270 0.1.1, crsf 2.0.1
