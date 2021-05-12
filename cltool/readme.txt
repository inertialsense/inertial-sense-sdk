Inertial Sense Command Line Tool (cltool) Readme


======================================
Linux Instructions

1. Create build directory...

   $ cd cltool
   $ mkdir build

2. Run cmake from within build directory.

   $ cd build
   $ cmake ..

   To cross-compile 32 or 64 bit:

   $ sudo apt-get install libc6-dev-i386
   $ sudo apt-get install gcc-multilib g++-multilib
   
   32 bit
   
   $ cmake .. -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32

   64 bit

   $ cmake .. -DCMAKE_CXX_FLAGS=-m64 -DCMAKE_C_FLAGS=-m64

3. Compile using make.

   $ make

4. If necessary, add current user to the "dialout" group to read and write to the USB serial communication ports.  In some cases the Modem Manager must be disabled to prevent interference with serial communication. 

   $ sudo usermod -a -G dialout $USER
   $ sudo usermod -a -G plugdev $USER
   $ sudo systemctl disable ModemManager.service && sudo systemctl stop ModemManager.service
   (reboot computer)

5. Run tool executable

   $ ./bin/cltool

======================================
Windows Instructions (MS Visual Studio)

1. Open Visual Studio solution file (VS_project/cltool.sln).

2. Build and run (F5).


======================================
Notes / FAQs



 