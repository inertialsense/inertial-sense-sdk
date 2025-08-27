#!/bin/bash

loop_count=1
curP=/dev/ttyACM1

exec > >(tee -a myscript.log) 2>&1
# Now everything below this line is logged to myscript.log

while true; do
    echo "=== Loop iteration: $loop_count ==="
    
    sleep 2

    ./cltool -dboc -c "$curP" '-flashCfg=startupGPSDtMs=500|startupImuDtMs=1|startupNavDtMs=20|gnssCn0Minimum=20|gnssCn0DynMinOffset=15|gps1AntOffset[0]=-0.50|gps1AntOffset[1]=0.00|gps1AntOffset[2]=0|gps2AntOffset[0]=0.50|gps2AntOffset[1]=-0.0|gps2AntOffset[2]=0|ioConfig=0x06DB2046|RTKCfgBits=0x00000008|dynamicModel=8|sysCfgBits=0x00000100|sensorConfig=0x01000033|gnssSatSigConst=0x0033|gpsTimeUserDelay=0|gpsMinimumElevation=0.17453292519943295|ser2BaudRate=38400'

    sleep 1

    ./cltool -dboc -c "$curP" -sysCmd=38
    sleep 1

    ./cltool -dboc -c "$curP" -reset
    sleep 3

    echo "Checking RTK compassing status..."
    ./cltool -c "$curP" -flashCfg | grep -i rtk | while read -r line; do
        echo "$line"
        if [[ "$line" == "RTKCfgBits = 0x00000000" ]]; then
            echo "RTK Compassing is disabled!"
            exit 0
        else
            echo "RTK Compassing is enabled!"
        fi
    done

    ((loop_count++))
done
