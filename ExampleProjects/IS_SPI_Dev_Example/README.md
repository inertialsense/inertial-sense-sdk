# SDK: Inertial Sense SPI Developement Example

## Software Requirements

- Microchip Studio (https://www.microchip.com/en-us/tools-resources/develop/microchip-studio)

## Hardware Requirements

* [IMX-5] (https://docs.inertialsense.com/user-manual/hardware/module_imx5/)
* [Sparkfun SAMD21 Mini Breakout](https://www.sparkfun.com/sparkfun-samd21-mini-breakout.html?gad_campaignid=17479024039&gad_source=1&gbraid=0AAAAADsj4ER3PNTf-q2T8ZxgVC8GTlCD2)
* [Atmel Ice](https://www.microchip.com/en-us/development-tool/atatmel-ice)

## Files
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\IS_SPI_example.atstart'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\IS_SPI_Example.h'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\IS_SPI_Example.c'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\atmel_start.h'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\main.c'
    '\SDK\src\ISComm.c'
    '\SDK\src\ISComm.h'
    '\SDK\src\ISConstants.h'
    '\SDK\src\data_sets.c'
    '\SDK\src\data_sets.h'
    '\SDK\src\rtk_defines.h'

## Steps to generate project

- Navigate to Atmel's project configuration site (https://start.atmel.com/).
- Click 'Load Project From File'.
- Browse to and select 'IS_SPI_example.atstart'.
- Click 'Click Open Selected File'.
- Click 'Export Project'.
- Name the project 'IS_SPI_example'
- Click 'Download Pack'.
- Open the downloaded 'IS_SPI_example.atzip' file.
- Click 'Ok' in the import project window.
- Delete the following files from the project directory.
    'atmel_start.h'
    'main.c'
    'usb_start.c'
    'usb_start.h'
- Add the follow files from the Inertial Sense SDK to the project directory.
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\IS_SPI_Example.h'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\IS_SPI_Example.c'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\atmel_start.h'
    '\SDK\ExampleProjects\IS_SPI_Dev_Example\main.c'
    '\SDK\src\ISComm.c'
    '\SDK\src\ISComm.h'
    '\SDK\src\ISConstants.h'
    '\SDK\src\data_sets.c'
    '\SDK\src\data_sets.h'
    '\SDK\src\rtk_defines.h'
- Open 'IS_SPI_example.atsln'.
- In solution Explorer. Right click on 'usb_start.c' and 'usb_start.h' and click 'Remove'
- In solution Explorer. Right click on 'IS_SPI_example' and click 'Add->Existing Item'
- Select the following files then click 'add'.
    'IS_SPI_Example.h'
    'IS_SPI_Example.c'
    'ISComm.c'
    'ISComm.h'
    'ISConstants.h'
    'data_sets.c'
    'data_sets.h'
    'rtk_defines.h'
- In solution Explorer. Right click on 'IS_SPI_example' and click 'Properties'.
- Navigate to 'Toolchain -> ARM/GNU C Compiler -> Symbols'.
- Click 'Add Item' under -D options.
- Add the following symbols:
    'SAMD_ZERO'
    'ARM'
- Save the properties.
- Build Solution.





