---
pagetitle: Release Notes for MotionMC Library
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="_htmresc/favicon.png" />
---

::: {.row}
::: {.col-sm-12 .col-lg-4}

<center>
# Release Notes for MotionMC Magnetometer Calibration Library
Copyright &copy; 2022 STMicroelectronics\

[![ST logo](_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>

# Purpose

**What is MotionMC?**

- The MotionMC provides real-time magnetometer calibration.
- The library acquires data from the magnetometer and counts the hard iron (HI) and scale factor (SF) (diagonal elements of soft iron matrix) coefficients together with the calibration quality value. The HI and SF coefficients are then used to compensate raw data from the magnetometer.

The library is generated for each ARM Cortex-M core with EABI compiler thus the library might be used with any other compiler which supports ARM EABI specification.

The library is available for two different wchar variable lengths - 16 bit (wc16) and 32bit (wc32).

By default the libraries are compiled with option -mfloat-abi=softfp which allows use of floating-point instructions but maintains compatibility with the soft-float ABI. The libraries ending with suffix hard are compiled with option -mfloat-abi=hard and uses floating-point instructions and the floating-point ABI.

Here is the list of references to the user documents:

- [Getting started with MotionMC magnetometer calibration library in X-CUBE-MEMS1 expansion for STM32Cube](https://www.st.com/resource/en/user_manual/dm00382798-getting-started-with-motionmc-magnetometer-calibration-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf)
- [Getting started with the X-CUBE-MEMS1 motion MEMS and environmental sensor software expansion for STM32Cube](https://www.st.com/resource/en/user_manual/dm00157069-getting-started-with-the-xcubemems1-motion-mems-and-environmental-sensor-software-expansion-for-stm32cube-stmicroelectronics.pdf)
:::

::: {.col-sm-12 .col-lg-8}
# Update history

::: {.collapse}
<input type="checkbox" id="collapse-section12" checked aria-hidden="true">
<label for="collapse-section12" aria-hidden="true">V2.5.1 / 10-May-2021</label>
<div>			

## Main changes

### Patch release

- Changed FPU DP to SP for CM7

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.9
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.32.0
- STM32CubeIDE V1.7.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section11" aria-hidden="true">
<label for="collapse-section11" aria-hidden="true">V2.5.0 / 06-May-2021</label>
<div>			

## Main changes

### Maintenance release

- Added CM33F support
- Updated Release Notes to new format

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.9
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.32.0
- STM32CubeIDE V1.6.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section10" aria-hidden="true">
<label for="collapse-section10" aria-hidden="true">V2.4.0 / 16-December-2020</label>
<div>			

## Main changes

### Maintenance release

- Update STM32 device check
- Add BlueNRG-LP device check
- Update library algorithm and interface

## Development toolchains and compilers

- STM32CubeIDE V1.5.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section9" aria-hidden="true">
<label for="collapse-section9" aria-hidden="true">V2.3.1 / 19-June-2020</label>
<div>			

## Main changes

### Patch release

- Library compiled in STM32CubeIDE

## Development toolchains and compilers

- STM32CubeIDE V1.3.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section8" aria-hidden="true">
<label for="collapse-section8" aria-hidden="true">V2.3.0 / 10-February-2020</label>
<div>			

## Main changes

### Maintenance release

- Added Cortex-M7 support
- Updated STM32 CRC Lock

## Development toolchains and compilers

- System Workbench for STM32 V2.7.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section6" aria-hidden="true">
<label for="collapse-section6" aria-hidden="true">V2.2.1 / 15-February-2019</label>
<div>			

## Main changes

### Patch release

- Unification of the libraries to single EABI library per Cortex core and wchar size

## Development toolchains and compilers

- System Workbench for STM32 V2.6.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section7" aria-hidden="true">
<label for="collapse-section7" aria-hidden="true">V2.2.0 / 20-June-2018</label>
<div>			

## Main changes

### Maintenance release

- Cortex-M0+ library update to be able to run also on BlueNRG-1 and BlueNRG-2 devices
- Remove dynamic memory allocation

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- IAR Embedded Workbench for ARM (EWARM) toolchain V8.11.2
- uVision(MDK-ARM) toolchain 5.22
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section5" aria-hidden="true">
<label for="collapse-section5" aria-hidden="true">V2.1.0 / 20-March-2018</label>
<div>			

## Main changes

### Maintenance release

- Add library and dedicated header file (motion_mc_cm0p.h) for Cortex-M0+

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- IAR Embedded Workbench for ARM (EWARM) toolchain V8.11.2
- uVision(MDK-ARM) toolchain 5.22
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section3" aria-hidden="true">
<label for="collapse-section3" aria-hidden="true">V2.0.1 / 01-September-2017</label>
<div>			

## Main changes

### Patch release

- Add library for EWARM 8 IDE. Due to the changes in EWARM 8 dedicated library for this environment must be used. The name of the library for EWARM 8 ends by _IAR8_ot.

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- IAR Embedded Workbench for ARM (EWARM) toolchain V8.11.2
- uVision(MDK-ARM) toolchain 5.22
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section2" aria-hidden="true">
<label for="collapse-section2" aria-hidden="true">V2.0.0 / 01-May-2017</label>
<div>			

## Main changes

### Product update

- First version of the library published under the ODE initiative. Previously released as part of OSX framework.

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- uVision(MDK-ARM) toolchain 5.22
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section1" aria-hidden="true">
<label for="collapse-section1" aria-hidden="true">V1.0.0 / 01-September-2016</label>
<div>			

## Main changes

### First release

- First official release

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.60
- uVision(MDK-ARM) toolchain 5.21
- System Workbench for STM32 V1.10.0

</div>
:::

:::
:::

<footer class="sticky">
::: {.columns}
::: {.column width="95%"}
For complete documentation on **MotionMC** ,
visit: [X-CUBE-MEMS1](https://www.st.com/en/embedded-software/x-cube-mems1.html)
:::
::: {.column width="5%"}
<abbr title="Based on template cx566953 version 2.0">Info</abbr>
:::
:::
</footer>
