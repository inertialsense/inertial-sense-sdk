![Inertial Sense Logo](https://inertialsense.com/wp-content/uploads/2019/09/logo-1.png)

# Inertial Sense SDK

The [Inertial Sense software development kit (SDK)](https://github.com/inertialsense/inertial-sense-sdk) is hosted on GitHub.

**SDK** - The Inertial Sense open source software development kit provides quick integration for communication with the Inertial Sense product line, including the uINS, uAHRS, and uINS.  It includes data logger, math libraries, and serial port interface for Linux and Windows environments.   

**EvalTool executable** - Graphical Windows-based desktop program that allows you to explore and test functionality of the Inertial Sense products in real-time.  It has scrolling plots, 3D model representation, table views of all data, data logger, and firmware updating interface for the uINS, uAHRS, or uIMU. The EvalTool can simultaneously interface with multiple Inertial Sense devices.

**CLTool** - Command line utility that can be used to communicate, log data, and update firmware for Inertial Sense products.  Additionally, the cltool serves as example source code to demonstrate how to integrate the Inertial Sense SDK into your own source code.  The cltool can be compiled in Linux and Windows. 

**EVB-2** - Multi-purpose hardware evaluation and development kit for the uINS.  The EVB-2 includes the uINS-G2 with Dual GNSS, RTK heading / positioning, onboard logging to micro SD card, 915MHz XBee radio for RTK base corrections, WiFi and BLE interface, serial and SPI communications to uINS interface, and Microchip SAME70 processor as communications bridge and user project development environment.   

**ROS** - The `inertial-sense-sdk/ros` directory contains the [ROS wrapper node implementation](ros) for the Inertial Sense IMX product line.

### Documents

 * [User Manual, Datasheet, and Dimensions](http://docs.inertialsense.com/)
 * [Inertial Sense ROS Instructions](ros/README.md)

### Downloads

 * [SDK Example Projects]( https://github.com/inertialsense/inertial-sense-sdk/tree/release/ExampleProjects) - Source code projects that demonstrations of how to use the SDK.
 * [Software Releases](https://github.com/inertialsense/inertial-sense-sdk/releases) - uINS, uAHRS, uIMU, and EVB-2 firmware and application installers.
 * [SDK & CLTool Source Code](https://github.com/inertialsense/inertial-sense-sdk) - Open source SDK repository with command line tool and example C/C++ source code.

### Hardware Design Files

 * [IS-hdw repository](https://github.com/inertialsense/IS-hdw) - CAD models of our products and PCB design assets for integration.

### Support

 * Email - support@inertialsense.com

------

### Open Source License

**MIT LICENSE**

Copyright 2014-2023 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

------

(c) 2014-2023 Inertial Sense, Inc.
