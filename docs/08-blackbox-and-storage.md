# 8. Blackbox & Storage

Files: [`src/main/blackbox/`](https://github.com/betaflight/betaflight/tree/d593400a0/src/main/blackbox), [`src/main/io/flashfs.c`](https://github.com/betaflight/betaflight/blob/d593400a0/src/main/io/flashfs.c), [`src/main/io/asyncfatfs/`](https://github.com/betaflight/betaflight/tree/d593400a0/src/main/io/asyncfatfs), [`src/main/msc/`](https://github.com/betaflight/betaflight/tree/d593400a0/src/main/msc).

Blackbox runs **inside the PID loop** — `blackboxUpdate(currentTimeUs)` is called from `subTaskPidSubprocesses` (`fc/core.c:1479`), meaning logging content is written exactly in sync with the control loop with a per-loop byte budget (`blackboxHeaderBudget`, refilled from device free-space).

## Frame format

Encoder design: **intra + inter frames with per-field predictors**, then variable-byte / zigzag / tag packing.

```mermaid
flowchart TD
    S0["state machine blackboxState_e blackbox.c:350<br/>STOPPED → PREPARE_LOG_FILE → SEND_HEADER →<br/>SEND_*_FIELD_HEADER → SEND_SYSINFO → RUNNING ⇄ PAUSED →<br/>SHUTTING_DOWN → STOPPED"]
    S0 --> MF["Main frame blackboxMainState_t blackbox.c:369<br/>time, pid P/I/D/F/S per axis, rcCommand, setpoint,<br/>gyro (filtered + raw), acc, attitude quaternion,<br/>debug[], motor[], servo[], erpm, vbat, amperage,<br/>baro, mag, rangefinder, rssi, pitot"]
    MF --> ENC["writeIntraframe :678 (first frame after event / I-frame)<br/>writeInterframe :850 (delta vs prediction)"]
    ENC --> PRED["field predictors blackbox_fielddefs.h:80<br/>PREVIOUS · STRAIGHT_LINE · AVERAGE_2 ·<br/>MINTHROTTLE · MOTOR_0 · INC · HOME_COORD · 1500 · VBATREF"]
    ENC --> PRIM["primitive encoders blackbox_encoding.c<br/>blackboxWriteUnsignedVB :89 · zigZag signed :100<br/>tag-and-field 2/4/6/32-bit packs :129/:253/:375"]
    PRIM --> EVT["events blackboxLogEvent :1978<br/>FLIGHT_LOG_EVENT_*: SYNC_BEEP, FLIGHTMODE,<br/>LOG_END, ADJUSTMENT... with detection hooks (arming beep sync :2031,<br/>mode change detection :2044)"]
    EVT --> OUT["blackboxDeviceFlush / budgeted write"]
```

Field presence adapts: `flightLogFieldCondition_e` (motors present? RC logged? debug columns? RPM telemetry available?...) — a log from a 4-motor DShot bidir config has a different schema than a wing with GPS.

## Storage backends

One recorder, three media — decided in `blackboxDeviceOpen` (`blackbox_io.c:309`) based on `blackboxConfig()->device`:

```mermaid
flowchart TD
    BB["blackboxWrite* (byte stream)"] --> B{device}
    B -->|SERIAL| S1["OpenLog / OpenLager<br/>serial port FUNCTION_BLACKBOX"]
    B -->|FLASH| S2["flashfs io/flashfs.c:529 flashfsWrite<br/>append-only log FS with async page programming<br/>flashfsFlushAsync :405 over drivers/flash/* chips"]
    B -->|SDCARD| S3["asyncfatfs io/asyncfatfs/asyncfatfs.c<br/>afatfs_fopen :66 · afatfs_poll · afatfs_getFilesystemState<br/>→ drivers/sdcard.c (SDIO DMA or SPI transport)"]
    S1 & S2 & S3 --> BUD["blackbox_io.c budget scheduler:<br/>blackboxHeaderBudget + blackboxReplenishHeaderBudget :712<br/>(based on device free space)"]
    B -->|VIRTUAL| S4["blackbox_virtual.c (in-memory, SITL/tests)"]
```

- `TASK_MAIN` (fc/tasks.c:129) continuously polls `afatfs_poll()` and `flashfsEraseAsync()` so page ops never block the control loop.
- Flush strategy: `blackboxDeviceFlush` (`:222`) when budget allows; `blackboxDeviceFlushForce` (`:250`) at disarm (`blackboxFinish`).

## Reading logs back

```mermaid
flowchart TD
    R{readback path}
    R -->|MSP| A["MSP_DATAFLASH_READ<br/>→ mspFcDataFlashReadCommand msp/msp.c:3023<br/>→ serializeDataflashReadReply :580<br/>(legacy 128-byte chunks + new length+Huffman format :625-660)<br/>→ flashfsReadAbs"]
    R -->|USB MSC| B["CLI msc / MSP_REBOOT_MSC / CMS<br/>→ systemResetToMsc (drivers/usb_msc_common.c:141)<br/>→ reboot into USB mass storage"]
    B --> C["msc/emfat.c — emulated FAT volume from flashfs logs<br/>emfat_init_files :411 · emfat_find_log :310<br/>reads served by bblog_read_proc :249<br/>(LUN backed by usbd_storage_emfat.c)"]
    B2["real SD card log readback: usbd_storage_sd_spi.c / usbd_storage_sdio.c"]
    C --> EX["exit via button → systemResetFromMsc usb_msc_common.c:152"]
```

The **emfat trick** is worth appreciating: the FC has no FAT *writer* for NOR flash (flashfs is append-only), so instead of teaching the host how to read flashfs, the firmware *emulates* a FAT drive whose files are synthesized on the fly from flashfs content — one USB drawer, log files appear as normal `.bbl`.

## What gets logged — and when not

- `blackboxConfig()->p_denom / p_ratio` control the PID-loop logging denominator; the log starts at arm (`blackboxSetState` transitions + disarm event reasons from `flightLogDisarmReason_e` core.h:44-60).
- Headers are rate-limited (`blackboxHeaderBudget`) because extremely large headers on SD would starve the loop; `blackboxReplenishHeaderBudget` recovers space-aware budgets for serial/flash/SD devices.
- MSP can also read `MSP_ANALOG`-style live data; blackbox is strictly offline-ish (except via virtual device in tests).
- Debug modes: `debug_mode` selects which DEBUG_* slice gets logged into `debug[]`; the same slice also feeds OSD/telemetry (`sensors/gyro.c` DEBUG_GYRO_RAW, `adcinternal` DEBUG slots...).
