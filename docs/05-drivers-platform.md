# 5. Drivers & Platform Abstraction

Files: [`src/main/drivers/`](https://github.com/betaflight/betaflight/tree/d593400a0/src/main/drivers) (~160 files), [`src/platform/`](https://github.com/betaflight/betaflight/tree/d593400a0/src/platform) (9 families).

## The one rule

> **Portable code in `src/main/` never includes a vendor SDK header.**

Peripheral handles are **opaque pointer types**; per-MCU knowledge flows in via *compile-time traits* and *per-platform implementation files* chosen by the build system.

```mermaid
flowchart TB
    subgraph L0["App layer: fc/, flight/, sensors/, io/, cli/, msp/ ..."]
        APP["typical calls: motorWriteAll(), displayWrite(), busWriteRegister(), spiReadReg(), dmaAllocate()"]
    end
    subgraph L1["Mid-tier dispatch (vtable multiplexers, portable)"]
        MT["motor.c · serial.c · display.c · vtx_common.c · bus.c<br/>(pass-through wrappers over device vtables)"]
    end
    subgraph L2["Portable driver implementations"]
        DR["accgyro_mpu.c · barometer_bmp280.c · flash.c · sdcard.c<br/>dshot.c · dshot_command.c · dshot_bitbang_decode.c<br/>bus_spi.c (queue engine) · adc.c · timer_common.c · io.c · dma.c<br/>ws2811 encode, light_ws2811strip.c"]
    end
    subgraph L3["impl contracts"]
        IMPL["bus_spi_impl.h · bus_i2c_impl.h · serial_uart_impl.h ·<br/>dma_impl.h · dshot_dpwm.h · bus_octospi_impl.h<br/>(declare functions each platform must provide)"]
        OPAQUE["opaque resources: spiResource_t · i2cDevice_t · dmaResource_t ·<br/>timerResource_t · busQuadSpiResource_t"]
    end
    subgraph L4["Platform trees — src/platform/<FAMILY>/"]
        STM["STM32/<br/>bus_spi_ll.c · bus_spi_stdperiph.c (F4)<br/>bus_spi_hal2.c (C5/N6)<br/>serial_uart_ll.c · pwm_output_dshot_hal.c<br/>system_stm32f4xx.c · exti.c · dma_stm32XX.c"]
        COMMON["common/stm32/<br/>code shared by all ST derivatives:<br/>dshot_dpwm.c · pwm_output_dshot_shared.c<br/>dshot_bitbang_shared.c · bus_spi_hw.c<br/>bus_spi_pinconfig.c · adc_impl.c · io_impl.c"]
        VEND["APM32/ · AT32/ · GD32/ · PICO/ · ESP32/ · X32/"]
        SIM["SIMULATOR/<br/>sitl.c · udplink.c · serial_tcp.c<br/>virtual sensors"]
        TRAITS["include/platform/platform.h<br/>SPI_TRAIT_AF_PIN · UART_TRAIT_* · DMA_TRAIT_CHANNEL<br/>PLATFORM_TRAIT_RCC · buffer placement attrs"]
    end
    L0 --> L1 --> L2 --> IMPL --> STM & COMMON & VEND & SIM
    IMPL --> OPAQUE
    TRAITS --> STM & COMMON
```

Which actual `.c` files get compiled is decided **purely by the per-MCU `.mk` source lists** (e.g. `STM32/mk/STM32G4.mk` pulls `bus_spi_ll.c + pwm_output_dshot_hal.c`, while `STM32F4.mk` pulls `bus_spi_stdperiph.c + serial_uart_stm32f4xx.c`) — there is no `#ifdef` switching between competing implementations inside shared files; `USE_` macros and *trait* macros instead gate filler content inside shared code.

## The three flavors of "vtable"

### A. Runtime device vtables (most common)

```mermaid
flowchart LR
    subgraph motor.c["drivers/motor.c (dispatch)"]
        W["motorWriteAll() motor.c:79"]
        S["motorDevInit() motor.c:299"]
    end
    subgraph vtables
        D1["dshotPwmVTable<br/>(STM32 timer-DMA)"]
        D2["dshotBitbangDevInit<br/>(DMA→GPIO bitbang)"]
        D3["motorPwmDevInit<br/>(analog PWM/OneShot/Multishot)"]
        D4["dronecanMotorDevInit<br/>(CAN ESC)"]
        D5["motorNullVTable<br/>(no motors)"]
    end
    S --> D1 & D2 & D3 & D4 & D5
    W -->|"updates via vTable.write / updateComplete"| vtables
```

`motorVTable_t` has 18 pointers (`motor_types.h:68-90`): `updateInit, write, writeInt, updateComplete, telemetryWait, decodeTelemetry, convertExternalToMotor, convertMotorToExternal, enable/disable, shutdown, isMotorIdle, getMotorIO…`. Same pattern for serial (`serialPortVTable` — 11 pointers, `serial_uart.c:565` fills `uartVTable`), display (`displayPortVTable_t`), VTX (`vtxVTable_s`), LCD panels, DroneCAN ESC.

### B. Per-instance function pointers (sensor style)

Older but still the pattern for sensors: shared structs expose `initFn/readFn` slots, each driver fills them:

```c
// drivers/accgyro/accgyro_spi_bmi270.c:569-582
acc->initFn = bmi270SpiAccInit;  acc->readFn = bmi270AccRead;
gyro->initFn = bmi270SpiGyroInit; gyro->readFn = bmi270GyroRead;
```

Same idea for `baroDev_t` (`drivers/barometer/barometer.h:33-46`) with its start_ut/read_ut/get_ut/start_up/read_up/get_up/calculate pipeline shaped around pressure & temperature read sequences.

### C. Compile-time "trait" polymorphism (per-MCU, zero overhead)

`platform/STM32/include/platform/platform.h:711-786` defines e.g. `SPI_TRAIT_AF_PIN`, `I2C_TRAIT_STATE`, `UART_TRAIT_*`, `DMA_TRAIT_CHANNEL/MUX`, `PLATFORM_TRAIT_RCC`, consumed by shared struct definitions in `*_impl.h` — templates without functions.

## IO, resources, and pin ownership

The key insight: **pins are owned**. Each physical pin is an `ioRec_t` in `ioRecs[]` (`drivers/io.c:82`); `IOInit(io, owner, index)` claims it:

```mermaid
flowchart LR
    TGT["target/board config<br/>pin tags: e.g. GYRO_CS ← DEFIO_TAG..."] -->|"IOGetByTag(ioTag)"| REC["ioRecs[] io.c:82<br/>one record per pin"]
    REC -->|"IOInit(owner, index)"| OWN["resourceOwner_e (~120 owners)<br/>OWNER_MOTOR · OWNER_GYRO_CS · OWNER_SPI_SCK/SDI/SDO · OWNER_QUADSPI_* ..."]
    OWN --> CLI["CLI 'resource' command<br/>resourceInputBase/Index (resource.c:160-201)<br/>prints owner names + index"]
    PREINIT["io_preinit.c:25 ioPreinitByIO<br/>OWNER_PREINIT + drive high before drivers<br/>(e.g. CS lines, OSD preinit)"] --> REC
```

- Timers: `timerAllocate(ioTag, owner, resourceIndex)` (`timer_common.c:112`) — configurable via the `timerio` PG (`USE_TIMER_MGMT`), with legacy auto-search fallback.
- DMA: one-shot claim via `dmaAllocate(identifier, owner, resourceIndex)` (`dma.c:26`); handlers installed later via `dmaSetHandler`. Request mapping tables live per-MCU (`platform/<FAM>/dma_reqmap_mcu.c`) and feed `dmaGetChannelSpecByPeripheral/ByTimer`; per-device `dmaopt` settings (PG) pin a specific mapping code.
- EXTI sharing: 16 EXTI lines each get one handler record; lines 5–9 (and 10–15 on some families) share an IRQ group whose NVIC priority is the **strictest requested** (`platform/STM32/exti.c:186-190`) — the mechanism that lets gyro + barometer-CS int + OSD chip int coexist.

## The bus subsystem (SPI/quad SPI/OCTOSPI/I2C)

```mermaid
sequenceDiagram
    autonumber
    participant D as Driver (e.g. barometer_bmp280.c)
    participant B as bus.c / bus_spi.c (queue engine)
    participant HW as Platform SPI impl (bus_spi_ll.c / stdperiph / hal2)
    participant IRQ as DMA/IRQ

    D->>B: spiSequence(dev, segments[])  bus_spi.c:460
    alt bus busy
        B->>B: append to trailing link segment (chained transfer list)
    else free
        B->>HW: spiSequenceStart(dev) → program first segment (DMA init cached per device)
        HW->>IRQ: DMA runs segment
        IRQ->>B: spiIrqHandler(dev) bus_spi.c:574
        B->>D: segment callback: BUS_READY → advance · BUS_BUSY → repeat · BUS_ABORT → skip
        B->>HW: pre-init next segment (spiPreInitStream) to cut inter-segment latency
        alt linked device next in list
            B->>HW: spiSequenceStart(nextDev) — CS of first device stays asserted semantics (negateCS)
        else list done
            B->>B: curSegment = BUS_SPI_FREE
        end
    end
```

- Transaction **segments** (`busSegment_t`) may hold buffers (`txData/rxData`), lengths with width nibbles, CS-negation flags, a per-segment callback, or a **link** to another device's segment list — enabling multi-device transaction chaining without CPU involvement between them.
- Polling path mirrors the same walk (`spiProcessSegmentsPolled`).
- High-level helpers `busWriteRegister / busReadRegisterBuffer(Start) / busBusy` (`drivers/bus.c`) adapt between SPI (0x7f/0x80 register bit, immediate completion + optional DMA) and I2C (async start/busy semantics), so sensor code is bus-agnostic.
- I2C has its own dev glue (`bus_i2c_busdev.c`) with blocking/async primitives and a soft-I2C bitbang option; QSPI/OCTOSPI exist for external flash (including **memory-mapped mode** used by MX66UW1G45G OPI flash for config-in-flash).

## DShot / motor output internals

```mermaid
flowchart TD
    A["mixTable() produces motor[] (normalized 0..1)"] --> B["motorWriteAll drivers/motor.c:79<br/>(per frame)"]
    B --> C{"protocol family"}
    C -->|DSHOT via timer DMA| D["dshot_dpwm.c (platform/common/stm32)<br/>dshotWrite → loadDmaBufferDshot/Proshot<br/>pwmCompleteDshotMotorUpdate"]
    C -->|DSHOT bitbang| E["dshot_bitbang_shared.c<br/>DMA→GPIO waveform, pacer timer"]
    C -->|analog| F["pwm_output per platform (e.g. STM32/pwm_output_hw.c)"]
    C -->|DroneCAN| G["dronecan ESC: RawCommand publish"]
    D --> H["After TX DMA completes:<br/>pwmDshotSetDirectionInput — same stream reconfigured<br/>as memory-to-memory GCR edge capture (MAX_GCR_EDGES 22)"]
    H --> I["pwmTelemetryDecode pwm_output_dshot_shared.c:260<br/>→ eRPM or EDT (temp/current/voltage) packet"]
    I --> J["dshotTelemetryState.motorState[] + motorFreqLpf<br/>(drivers/dshot.c updateDshotTelemetry, quality tracking)"]
    J --> K["rpm filter (flight/rpm_filter.c) + esc_sensor (io/esc_sensor.c) +<br/>OSD/MSP (MOTOR_TELEMETRY) consumers"]
    L["dshotCommandWrite dshot_command.c:185<br/>BLOCKING (calibration, direction, save, beacons)<br/>INLINE (EDT enable, streamed into frames)"] --> D
```

Note the F4 errata workaround: DShot bitbang uses DMA2, so SPI DMA on DMA2 is disabled when bitbang is active (`bus_spi_hw.c:215-238`).

## Flash-backed abstractions

- `drivers/flash/flash.c`: NOR/NAND chips (m25p16/w25n/w25m/mt29f NAND/MX66UW1G45G OPI — over SPI/QSPI/OCTOSPI). Above it sits partitioning (`flash_partitionFindByType`) supporting **config-in-flash (EXST / CONFIG_IN_EXTERNAL or MEMORY_MAPPED_FLASH)** + BlackBox partition + Bootloader requests.
- `drivers/sdcard.c` + `sdcard_sdio_baremetal.c` + `sdcard_spi.c` + `sdcard_standard.c`: SD card front with a state machine (`SDCARD_STATE_*`), two transports (SDIO DMA / SPI). Used by config (`CONFIG.BIN`), BlackBox logging (FAT via `io/asyncfatfs`), MSC.

## Build-time wiring

MCU `.mk` files define `MCU_COMMON_SRC`, `DEVICE_STDPERIPH_SRC`, `VCP_SRC`, `STARTUP_SRC`, `LINKER_SCRIPT_SRC`, plus `TARGET_(HSE_MHZ|MCU|MCU_FAMILY)`. There is **no features.mk** — feature gating is by `USE_*` defines from target/board headers and `common_pre.h`/`common_defaults_post.h`, and at runtime by `featuresSupportedByBuild` (`config/feature.c:38`) so a feature configured by the user is rejected if its driver wasn't compiled in.
