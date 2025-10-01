---
pagetitle: Release Notes for MotionAR Library
lang: en
header-includes: <link rel="icon" type="image/x-icon" href="_htmresc/favicon.png" />
---

::: {.row}
::: {.col-sm-12 .col-lg-4}

<center>
# Release Notes for MotionAR Activity Recognition Library
Copyright &copy; 2022 STMicroelectronics\

[![ST logo](_htmresc/st_logo_2020.png)](https://www.st.com){.logo}
</center>

# Purpose

**What is MotionAR?**

- The MotionAR provides real-time information on the type of activity performed by the user.
- The library acquires data from the accelerometer and provides information on the type of activity performed by the user. It is able to distinguish the following activities: stationary, walking, fast walking, jogging, biking, driving.

The library is generated for each ARM Cortex-M core with EABI compiler thus the library might be used with any other compiler which supports ARM EABI specification.

The library is available for two different wchar variable lengths - 16 bit (wc16) and 32bit (wc32).

By default the libraries are compiled with option -mfloat-abi=softfp which allows use of floating-point instructions but maintains compatibility with the soft-float ABI. The libraries ending with suffix hard are compiled with option -mfloat-abi=hard and uses floating-point instructions and the floating-point ABI.

Here is the list of references to the user documents:

- [Getting started with MotionAR activity recognition library in X-CUBE-MEMS1 expansion for STM32Cube](https://www.st.com/resource/en/user_manual/dm00383387-getting-started-with-motionar-activity-recognition-library-in-xcubemems1-expansion-for-stm32cube-stmicroelectronics.pdf)
- [Getting started with the X-CUBE-MEMS1 motion MEMS and environmental sensor software expansion for STM32Cube](https://www.st.com/resource/en/user_manual/dm00157069-getting-started-with-the-xcubemems1-motion-mems-and-environmental-sensor-software-expansion-for-stm32cube-stmicroelectronics.pdf)
:::

::: {.col-sm-12 .col-lg-8}
# Update history

::: {.collapse}
<input type="checkbox" id="collapse-section12" checked aria-hidden="true">
<label for="collapse-section12" aria-hidden="true">V3.2.1 / 24-August-2021</label>
<div>			

## Main changes

### Patch release

- Changed F767 CPU to F722 in order to switch FPU DP to SP for CM7

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V8.50.9
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.32.0
- STM32CubeIDE V1.7.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section11" aria-hidden="true">
<label for="collapse-section11" aria-hidden="true">V3.2.0 / 28-April-2021</label>
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
<label for="collapse-section10" aria-hidden="true">V3.1.0 / 13-November-2020</label>
<div>			

## Main changes

### Maintenance release

- Library compiled in STM32CubeIDE
- Add hard FP option for some of library binaries
- Add Cortex-M7 support

## Development toolchains and compilers

- STM32CubeIDE V1.3.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section9" aria-hidden="true">
<label for="collapse-section9" aria-hidden="true">V3.0.2 / 11-June-2019</label>
<div>			

## Main changes

### Patch release

- Fixed the issue of interpolation scale when timestamp exceed 28 bit value
- Replaced long int with int64_t

## Development toolchains and compilers

- System Workbench for STM32 V2.7.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section8" aria-hidden="true">
<label for="collapse-section8" aria-hidden="true">V3.0.1 / 15-February-2019</label>
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
<label for="collapse-section7" aria-hidden="true">V3.0.0 / 01-September-2018</label>
<div>			

## Main changes

### Product update

- Improved library performance

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- IAR Embedded Workbench for ARM (EWARM) toolchain V8.20.2
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.24.0
- System Workbench for STM32 V2.6.0

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section6" aria-hidden="true">
<label for="collapse-section6" aria-hidden="true">V2.0.1 / 01-September-2017</label>
<div>			

## Main changes

### Patch release

- Add library for EWARM 8 IDE. Due to the changes in EWARM 8 dedicated library for this environment must be used
- The name of the library for EWARM 8 ends by _IAR8_ot

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- IAR Embedded Workbench for ARM (EWARM) toolchain V8.11.2
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.22.0
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section5" aria-hidden="true">
<label for="collapse-section5" aria-hidden="true">V2.0.0 / 01-May-2017</label>
<div>			

## Main changes

### Product update

- First version of the library published under the ODE initiative
- Previously released as part of OSX framework

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.22.0
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section4" aria-hidden="true">
<label for="collapse-section4" aria-hidden="true">V1.2.0 / 04-April-2016</label>
<div>			

## Main changes

### Maintenance release

- Improved performances in case of biking and driving activities

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.22.0
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section3" aria-hidden="true">
<label for="collapse-section3" aria-hidden="true">V1.1.0 / 15-December-2015</label>
<div>			

## Main changes

### Maintenance release

- Initialize function return value

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.22.0
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section2" aria-hidden="true">
<label for="collapse-section2" aria-hidden="true">V1.0.1 / 15-September-2015</label>
<div>			

## Main changes

### Patch release

- Production License supported

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.80.4
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.22.0
- System Workbench for STM32 V1.13.1

</div>
:::

::: {.collapse}
<input type="checkbox" id="collapse-section1" aria-hidden="true">
<label for="collapse-section1" aria-hidden="true">V1.0.0 / 08-June-2015</label>
<div>			

## Main changes

### First release

- First official release

## Development toolchains and compilers

- IAR Embedded Workbench for ARM (EWARM) toolchain V7.30.0
- RealView Microcontroller Development Kit (MDK-ARM) toolchain V5.14.0
- System Workbench for STM32 V1.1.0

</div>
:::

:::
:::

<footer class="sticky">
::: {.columns}
::: {.column width="95%"}
For complete documentation on **MotionAR** ,
visit: [X-CUBE-MEMS1](https://www.st.com/en/embedded-software/x-cube-mems1.html)
:::
::: {.column width="5%"}
<abbr title="Based on template cx566953 version 2.0">Info</abbr>
:::
:::
</footer>
