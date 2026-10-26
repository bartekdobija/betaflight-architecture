# 9. Configuration System

Files: [`src/main/pg/`](../betaflight/src/main/pg), [`src/main/config/`](../betaflight/src/main/config), [`src/main/cli/settings.c`](../betaflight/src/main/cli/settings.c).

Every persisted setting in Betaflight is a **Parameter Group (PG)**: a struct registered into a linker section at build time. EEPROM persistence, CLI dump, and MSP all operate on the same structs — there is no duplicated configuration model.

## The PG registry

```mermaid
flowchart TB
    subgraph BUILD["Build time"]
        REG["PG_REGISTER<...* macros pg/pg.h:119-176<br/>+ PG_DECLARE(_type,_name) → _System & _Copy structs<br/>+ accessors name() / nameMutable()"]
        RESET["PG_REGISTER_WITH_RESET_TEMPLATE / _WITH_RESET_FN<br/>templates live in section .pg_resetdata<br/>reset functions call pgResetFn_xxx"]
        SEC[".pg_registry linker section (platform link scripts)<br/>pgRegistry_t { pgn(12bit id + 4bit version) · length · size ·<br/>address(RAM) · copy(defaults shadow) · reset · fnv_hash ptr }"]
    end
    subgraph RUN["Runtime (pg/pg.c)"]
        FIND["pgFind(pgn) :32 linear scan"]
        RESETFN["pgResetInstance(reg, base) :47<br/>memset 0 → memcpy template or call reset.fn"]
        LOAD["pgLoad(reg, from, size, version) :76<br/>reset first, restore only if stored version == pgVersion"]
        STORE["pgStore(reg, to, size) :92"]
        RESETALL["pgResetAll / pgResetCopy :99/:66"]
    end
    SEC --> FIND
    BUILD --> RUN
```

Key macros and their meanings:

| Macro | Meaning |
|---|---|
| `PG_REGISTER(type, name, pgn, version)` | simple no-reset PG |
| `PG_REGISTER_WITH_RESET_FN(type, name, pgn, version)` | provides `pgResetFn_##name()` |
| `PG_REGISTER_WITH_RESET_TEMPLATE(...)` + `PG_RESET_TEMPLATE` | defaults are a literal struct blob in `.pg_resetdata` |
| `PG_REGISTER_ARRAY[_WITH_RESET_FN](type, count, name, pgn, version)` | per-instance array (profiles, serial ports, gyro devices…) |
| `PG_DECLARE_ARRAY(_type, maxLength, name)` | array accessors `name(i)`/`nameMutable(i)` |

PG IDs are a **hand-maintained registry** in [`pg/pg_ids.h`](../betaflight/src/main/pg/pg_ids.h): `PG_FAILSAFE_CONFIG 1`, `PG_MOTOR_CONFIG 6`, `PG_PID_PROFILE 14`, `PG_SYSTEM_CONFIG 18`, `PG_FEATURE_CONFIG 19`, `PG_RX_CONFIG 24`, … They are part of the **on-flash format contract** and are checked for uniqueness ordering by a unit test (`src/test/unit/check_pg_ids.awk` wired into `make test`).

The **4-bit version** stored in the id's top bits drives migration: on EEPROM load, a PG whose stored version ≠ current is dropped to defaults rather than misread — this is how fields get added/removed safely without config breaks.

## EEPROM persistence

```mermaid
flowchart LR
    subgraph BOOT["Boot: initEEPROM fc/config_eeprom.c:289"]
        AB{backing store?}
        AB -->|file| F1["CONFIG_IN_FILE (SITL/EXST file)"]
        AB -->|embedded| F2["configUnlock → configWriteWord pages<br/>(internal flash)"]
        AB -->|extern| F3["CONFIG_IN_EXTERNAL_FLASH / MEMORY_MAPPED_FLASH<br/>via flashPartitionFindByType(FLASH_PARTITION_TYPE_CONFIG)<br/>(incl. MX66UW1G45G OPI memory-mapped mode)"]
        AB -->|sdcard| F4["CONFIG_IN_SDCARD → CONFIG.BIN via afatfs"]
    end
    LOAD["loadEEPROM :429 — PG_FOREACH → findEEPROM(reg, class)<br/>→ pgLoad or pgReset → FNV1a hash fill"]
    VAL["isEEPROMVersionValid / isEEPROMStructureValid :339<br/>+ diagnoseEEPROM distinguishes EMPTY/CORRUPT/VERSION/PG_MISSING/PG_VERSION/BOARD"]
    WR["writeSettingsToEEPROM :487 — header, then one configRecord_t per PG<br/>{uint16 size · pgn · version · flags} · footer · big-endian CRC-16/CCITT"]
    STREAM["config_streamer.c :40-189 — buffered word writes<br/>branches per backing (page program vs sector erase vs memcpy)"]
    AB --> LOAD --> VAL
    WR --> STREAM
```

- The **FNV-1a hash** per PG (filled by `pgLoad`, compared in `writeSettingsToEEPROM`) is the dirty-detection mechanism: unchanged PGs skip their EEPROM re-erase.
- `_SYSTEM` record-class flags (`CR_CLASSICATION_MASK=0x3`) are legacy; profiles are stored as *big array PGs* (`PID_PROFILE_COUNT 4`, `CONTROL_RATE_PROFILE_COUNT`, `BATTERY_PROFILE_COUNT 3` — from `common_pre.h:355-359`).

## Profile activation & runtime switching

```mermaid
flowchart TD
    A["readEEPROM config/config.c:775<br/>loadEEPROM → featureInit → validateAndFixConfig → activateConfig"]
    A --> B["currentPidProfile = pidProfilesMutable(pidProfileIndex) :142<br/>loadControlRateProfile / loadBatteryProfile :167<br/>re-init rc / pid / filters :194"]
    B --> C{later: profile switch}
    C -->|RC aux| D["processRxModes → changeProfile<br/>→ changePidProfile config.c:866"]
    C -->|cell count change| E["changePidProfileFromCellCount :897"]
    C -->|MSP / CLI| F["MSP_SELECT_SETTING / CLI profile command"]
    D & E & F --> G["repoint pidProfileIndex in systemConfig PG + activateConfig()"]
```

**Profile indirection lives at the CLI/MSP layer, not the PG layer**: `clivalue_t` rows declare `PROFILE_VALUE / PROFILE_RATE_VALUE / PROFILE_BATTERY_VALUE` flags; `getValueOffset` (`cli/cli.c:909`) adds `sizeof(pidProfile_t) * getPidProfileIndexToUse()` to reach the selected profile's copy of the struct.

## Features

`config/feature.c`:

- `featureConfig` PG stores a bitmask of *desired* features.
- `featuresSupportedByBuild` (:38-102) is the **build-time capability mask** — a feature may only be *configured* if its `USE_*` driver was compiled in (checked in `featureEnableImmediate` and CLI validation).
- Defaults: `DEFAULT_FEATURES | DEFAULT_RX_FEATURE | FEATURE_ANTI_GRAVITY | FEATURE_AIRMODE`.

### Tuning sliders (`config/simplified_tuning.c`)

With `USE_SIMPLIFIED_TUNING`, the Configurator's "Tuning" sliders (master gain, expo, D/D-Max, dynamic filter level, …) map to concrete setting values through `calculateNewPidValues()`/`calculateNewDTermFilterValues()`/`calculateNewGyroFilterValues()` in this module: it starts from the in-code defaults and scales the profile fields for each axis — a bidirectional bridge, so CLI `dump` still shows real PID/rate numbers. The slider *state* (which sliders are enabled + their values) is part of the pidProfile PG.

## CLI/MSP binding (the settings engine)

```mermaid
flowchart LR
    VT["clivalue_t valueTable cli/settings.c:778 (~2300 rows)<br/>{ name · type · config · pgn · offset }"]
    LUT["lookupTable*[] settings.c:166-760<br/>(enum-to-string tables for LOOKUP types)"]
    CLI["CLI cliGet/cliSet cli.c:5273/5489"] --> VT
    LUT --> CLI
    MSPC["MSP_SET_xxx handlers msp/msp.c"] --> VT2["same PG structs"]
    VT --> PGF["pgFind(pgn) → reg<br/>cliGetValuePointer cli.c:924 = reg->address + offset(+ profile offset)"]
    PGF --> RAM["live config structs in RAM"]
    DUMP["dump/diff cli/dumpAllValues :967<br/>compares live vs pg->copy (defaults shadow)"]
```

Note the three *views* over one data model:

1. **CLI** (text names) → `clivalue_t` → PG structs
2. **MSP** (numeric per-field setter commands, generated tables) → same PG structs
3. **EEPROM** (schema binary) → same PG structs (versioned)

## Where target/board-level config lives

- `targetConfiguration()` / `configTargetPreInit()` (declared `config/config.h:100-102`, called from `fc/init.c:305-312`) are hooks provided by the board's optional `config.c` in the `src/config` submodule — the mechanism for board-specific peripheral banks (e.g. assigning OSD SPI to SPI3, ADC channels, gyro devices).
- Defaults fixed at build time come from `target/common_pre.h` (feature gates, counts) and `common_defaults_post.h` (per-family hardware defaults such as `RX_MIN_USEC/RX_MAX_USEC`, I2C bus speeds, USB MSC button pins).

## Unit test hooks

- `src/test/unit/check_pg_ids.awk` — id registry consistency check
- gtest suites exercise PG scaling, EEPROM serialization (`unittest` C harnesses via `unittest_c.c`), and virtual blackbox device (`blackbox_virtual.c`)
