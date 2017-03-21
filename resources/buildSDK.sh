#!/bin/bash
echo Building x32 and x64 static libraries

rm -f linux.zip
rm -rf build
mkdir build
mkdir build/x32
mkdir build/x64


cd ..
cd InertialSenseCLTool/

rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS=-m32 -DCMAKE_C_FLAGS=-m32
make -j 7
cp bin/cltool ../../resources/build/x32/cltool
cp lib/libInertialSense.a ../../resources/build/x32/libInertialSense.a

cd ..
rm -rf build
mkdir build
cd build
cmake .. -DCMAKE_CXX_FLAGS=-m64 -DCMAKE_C_FLAGS=-m64
make -j 7
cp bin/cltool ../../resources/build/x64/cltool
cp lib/libInertialSense.a ../../resources/build/x64/libInertialSense.a

cd ../../resources

zip -r linux.zip build


