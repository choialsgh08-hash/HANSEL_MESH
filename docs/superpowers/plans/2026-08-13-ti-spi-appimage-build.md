# TI SPI ADC Appimage Build Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a flashable IWRL6432BOOST Motion/Presence appimage with TI's full-frame SPI ADC streaming enabled and enough frame-buffer memory for the current near-wall profile.

**Architecture:** Keep the installed MMWAVE-L-SDK immutable and build from a mechanically copied SDK tree under the workspace temporary area. Enable the SDK's existing `SPI_ADC_DATA_STREAMING` path through SysConfig, size its frame buffer from the checked-in radar profile, and use only TI's documented compiler, SysConfig, make, and boot-image tools. Preserve the generated appimage, linker map, build log, configuration diff, and hashes as evaluation artifacts; do not flash hardware in this task.

**Tech Stack:** MMWAVE-L-SDK 05.05.04.02, CCS 12.7.1, TI Arm Clang 3.2.2.LTS, SysConfig 1.20.0, GNU make, PowerShell.

## Global Constraints

- Target board/package is IWRL6432BOOST FCCSP using the `xwrL64xx-evm` Motion/Presence demo.
- The installed SDK under `C:/ti/MMWAVE_L_SDK_05_05_04_02` remains unchanged.
- The firmware must use TI's existing `adcLogging 2` full-frame SPI path; it must not attempt RDIF/DCA1000 capture.
- The checked-in UART firmware and current robot runtime remain untouched.
- Current profile geometry is 3 RX, 256 ADC samples, 2 chirps per burst, 16 bursts per frame, REAL16, giving 49,152 bytes per frame.
- This task proves compilation, linking, and appimage generation only; it does not claim successful electrical capture without a C232HM-DDHSL-0.

---

### Task 1: Reproduce the TI Build Environment

**Files:**
- Read: `C:/ti/MMWAVE_L_SDK_05_05_04_02/MMWAVE_L_SDK_05_05_04_02/imports.mak`
- Create: `tmp/ti-spi-appimage/build-environment.txt`

**Interfaces:**
- Consumes: SDK-required tool versions from `imports.mak`.
- Produces: Resolved executable paths for make, TI Arm Clang, SysConfig Node CLI, and boot-image tools.

- [ ] **Step 1: Record the required tool versions and currently resolved paths**

```powershell
Get-Content C:\ti\MMWAVE_L_SDK_05_05_04_02\MMWAVE_L_SDK_05_05_04_02\imports.mak
Get-Command gmake,make,tiarmclang -ErrorAction SilentlyContinue
```

- [ ] **Step 2: Install only missing official TI build dependencies**

Use CCS 12.7.1 containing `ti-cgt-armllvm_3.2.2.LTS` and SysConfig 1.20.0 at the paths expected by `imports.mak`. Do not substitute an unverified compiler version.

- [ ] **Step 3: Verify every executable directly**

```powershell
& C:\ti\ccs1271\ccs\utils\bin\gmake.exe --version
& C:\ti\ccs1271\ccs\tools\compiler\ti-cgt-armllvm_3.2.2.LTS\bin\tiarmclang.exe --version
& C:\ti\sysconfig_1.20.0\nodejs\node.exe C:\ti\sysconfig_1.20.0\dist\cli.js --help
```

Expected: every command exits 0 and identifies the required version.

### Task 2: Create and Configure an Isolated Firmware Build Tree

**Files:**
- Copy: `C:/ti/MMWAVE_L_SDK_05_05_04_02/MMWAVE_L_SDK_05_05_04_02` to `tmp/ti-spi-appimage/MMWAVE_L_SDK_05_05_04_02`
- Modify copy: `examples/mmw_demo/motion_and_presence_detection/xwrL64xx-evm/example.syscfg`
- Modify copy: `examples/mmw_demo/motion_and_presence_detection/source/mmw_cli.h`
- Read: `configs/radar/iwrl6432_2p5d_operator_nearwall_5hz.cfg`
- Create: `tmp/ti-spi-appimage/configuration.patch`

**Interfaces:**
- Consumes: Current profile frame dimensions and TI's `SPI_ADC_DATA_STREAMING` configuration field.
- Produces: An isolated source tree with SPI streaming enabled and `ADC_DATA_BUFF_MAX_SIZE >= 49152`.

- [ ] **Step 1: Copy the SDK and verify the source tree hash inputs**

```powershell
Copy-Item -Recurse -Force C:\ti\MMWAVE_L_SDK_05_05_04_02\MMWAVE_L_SDK_05_05_04_02 tmp\ti-spi-appimage\MMWAVE_L_SDK_05_05_04_02
```

- [ ] **Step 2: Add a preflight assertion that fails on the stock configuration**

```powershell
Select-String -Path <copied-example.syscfg> -Pattern 'SPI_ADC_DATA_STREAMING.*true'
Select-String -Path <copied-mmw_cli.h> -Pattern 'ADC_DATA_BUFF_MAX_SIZE.*49152'
```

Expected: at least one assertion fails before configuration changes.

- [ ] **Step 3: Enable SPI ADC streaming and set the full-frame buffer**

Set the SysConfig TI demo option that generates `SPI_ADC_DATA_STREAMING=1`. Set `ADC_DATA_BUFF_MAX_SIZE` to exactly 49,152 bytes unless alignment or linker requirements require a larger explicit multiple; document any larger value.

- [ ] **Step 4: Run the preflight assertion again**

Expected: both SPI enablement and frame-buffer assertions pass.

- [ ] **Step 5: Save a source configuration diff**

Create `tmp/ti-spi-appimage/configuration.patch` containing only the SysConfig and ADC buffer changes relative to the installed SDK.

### Task 3: Clean-Build the Release Appimage

**Files:**
- Build: `tmp/ti-spi-appimage/MMWAVE_L_SDK_05_05_04_02/examples/mmw_demo/motion_and_presence_detection/xwrL64xx-evm/m4fss0-0_freertos/ti-arm-clang`
- Create: `tmp/ti-spi-appimage/build-release.log`

**Interfaces:**
- Consumes: Resolved TI toolchain and configured firmware source.
- Produces: `.out`, `.map`, `.appimage`, and complete build log.

- [ ] **Step 1: Run a clean build**

```powershell
& C:\ti\ccs1271\ccs\utils\bin\gmake.exe clean PROFILE=release
& C:\ti\ccs1271\ccs\utils\bin\gmake.exe -j2 PROFILE=release
```

Capture stdout and stderr without hiding the process exit code.

- [ ] **Step 2: Verify the build output exists and is non-empty**

```powershell
Get-Item motion_and_presence_detection_demo.release.out
Get-Item motion_and_presence_detection_demo.release.appimage
Get-Item motion_and_presence_detection_demo.release.map
```

Expected: all three files exist, are non-zero, and the build exits 0.

### Task 4: Verify Firmware Configuration and Memory Fit

**Files:**
- Read: generated `ti_drivers_config.*`, linker `.map`, `.appimage`, build log.
- Create: `tmp/ti-spi-appimage/build-report.json`
- Create: `tmp/ti-spi-appimage/SHA256SUMS.txt`

**Interfaces:**
- Consumes: Successful Task 3 build artifacts.
- Produces: Machine-readable proof of SPI enablement, buffer sizing, memory use, tool versions, artifact sizes, and SHA-256 hashes.

- [ ] **Step 1: Confirm generated SPI symbols and compiled streaming code**

Verify generated configuration contains the MCSPI instance and the preprocessed/linked image contains `SPI_ADC_DATA_STREAMING=1` behavior. Treat a dead or missing SPI path as failure even if the generic demo links.

- [ ] **Step 2: Inspect linker memory usage**

Parse the `.map` file and record every memory region's used/free size. Fail if any section overflows or the 49,152-byte ADC buffer is absent.

- [ ] **Step 3: Hash all reproducibility inputs and outputs**

Hash the original SDK installer/source identity, current `.cfg`, modified SysConfig/header, `.out`, `.map`, and `.appimage` with SHA-256.

- [ ] **Step 4: Write the final build report**

Record target, SDK/tool versions, exact command, exit code, artifact paths/sizes/hashes, buffer size, expected bytes per frame, and memory headroom. State `hardware_capture_status: pending_cable`.

- [ ] **Step 5: Run a fresh final verification**

```powershell
& C:\ti\ccs1271\ccs\utils\bin\gmake.exe clean PROFILE=release
& C:\ti\ccs1271\ccs\utils\bin\gmake.exe -j2 PROFILE=release
```

Expected: exit 0, regenerated appimage has a stable SHA-256 for identical inputs, and the report validation exits 0.
