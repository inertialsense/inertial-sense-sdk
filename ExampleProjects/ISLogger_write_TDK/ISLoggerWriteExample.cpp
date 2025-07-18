// ConvertCsvToIsb.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <fstream>
#include <string>

#include "data_sets.h"
#include "ISLogger.h"
#include "filter.h"

// 800 sample * 60 sec * 60 minutes * 12 hours
//#define IN_DATA_MAX     100000
#define IN_DATA_MAX     40000000

#define BROAD_CAST_RATE 800
#define DELTA_TIME_MS   ((double)(1.0 / BROAD_CAST_RATE))

#define _X_     0
#define _Y_     1
#define _Z_     2


#define ACC_SCALE   (float)(-9.80665/2059.426)//.017453
#define GYRO_SCALE  (float)(-0.0612)


using namespace std;

void setStructDefaults();

ifstream inFile;
ifstream outFile;

string inFileName;
string outFileName;
string lines[];

pimu_t tmpPimu;
sensor_compensation_t tmpScomp;
dev_info_t tmpDevInfo;

int curTime = 0;
int parseCnt = 0;

typedef struct inData {
    int index;
    int accX;
    int accY;
    int accZ;
    int gyX;
    int gyY;
    int gyZ;
    double temperature;
    int timeMs;
}inData_t;

inData_t parsedData[IN_DATA_MAX];

int parseInputFile()
{
    string subStrings[8];
    string curLine;
    
    cout << "Reading file: " << inFileName << "\r\n";
    inFile.open(inFileName);

    // get header
    if (inFile.is_open() && inFile.good())
    {
        cout << "File open!\r\n";
        cout << "Parsing file.\r\n";
        getline(inFile, curLine);
    }

    while (inFile.is_open() && inFile.good() && parseCnt < IN_DATA_MAX)
    {
        getline(inFile, curLine);
        
        for (int i = 0, j = 0,subI = 0; i < curLine.size(); i++)
        {
            if (curLine[i] == ',')
            {
                subStrings[subI] = curLine.substr(j, i - j);
                subI++;
                j = i + 1;
            }
        }

        parsedData[parseCnt].index = stoi(subStrings[0]);
        parsedData[parseCnt].accX= stoi(subStrings[1]);
        parsedData[parseCnt].accY = stoi(subStrings[2]);
        parsedData[parseCnt].accZ = stoi(subStrings[3]);
        parsedData[parseCnt].gyX = stoi(subStrings[4]);
        parsedData[parseCnt].gyY = stoi(subStrings[5]);
        parsedData[parseCnt].gyZ = stoi(subStrings[6]);
        parsedData[parseCnt].temperature = stod(subStrings[7]);

        parsedData[parseCnt].timeMs = parseCnt * DELTA_TIME_MS * 1000;

        if (parseCnt % 100000 == 1)
            cout << "Parsed line: " << parseCnt << ".\r\n";

        parseCnt++;
    }

    cout << "Parse complete. Lines: " << parseCnt << " \r\n";

    return 0;
}

#define O0_LP_FILTER_local(val,input,alph,beta)					(val = (((beta)*(val)) + ((alph)*(input))))

// Low-pass filter
void step_lpf_sens_comp(sensor_comp_unit_t& scu, float alpha, float beta, float temp, float input[3])
{
    static uint32_t calStateLast = 1;

    if (calStateLast != tmpScomp.calState)
    {	// State change detected.  Initialize LPF.
        calStateLast = tmpScomp.calState;

        scu.lpfTemp = temp;

        for (int i = 0; i < 3; i++)
        {
            scu.lpfLsb[i] = input[i];
        }
    }
    else
    {	// Low-pass filter sensor data
        O0_LP_FILTER_local(scu.lpfTemp, temp, alpha, beta);

        for (int i = 0; i < 3; i++)
        {
            O0_LP_FILTER_local(scu.lpfLsb[i], input[i], alpha, beta);
        }
    }
}

#define ALPH_750SR_0p001CF  .000015 
#define BETA_750SR_0p001CF  .999985 // 1-ALPH_750SR_0p001CF

#define ALPH_750SR_0p01CF   .00015 
#define BETA_750SR_0p01CF   .99985 // 1-ALPH_750SR_0p001CF

void runScompLpf(int i)
{
    // convert input to propper value
    static sensors_w_temp_t input;

    // Assign temperature in C
    input.temp[0] = parsedData[i].temperature;
    input.temp[1] = parsedData[i].temperature;
    input.temp[2] = parsedData[i].temperature;

    // M/s^2
    input.imu3.I[0].acc[_X_] = parsedData[i].accX * .00048828125 * 9.80665;
    input.imu3.I[0].acc[_Y_] = parsedData[i].accY * .00048828125 * 9.80665;
    input.imu3.I[0].acc[_Z_] = parsedData[i].accZ * .00048828125 * 9.80665;

    // rad/s
    input.imu3.I[0].pqr[_X_] = parsedData[i].gyX * C_DEG2RAD * GYRO_SCALE;
    input.imu3.I[0].pqr[_Y_] = parsedData[i].gyY * C_DEG2RAD * GYRO_SCALE;
    input.imu3.I[0].pqr[_Z_] = parsedData[i].gyZ * C_DEG2RAD * GYRO_SCALE;

    // Run Low Pass Filter
    static uint32_t calStateLast = 1;

    if (calStateLast != tmpScomp.calState)
    {	// State change detected.  Initialize LPF.
        calStateLast = tmpScomp.calState;

        tmpScomp.acc[0].lpfTemp = input.temp[0];
        tmpScomp.pqr[0].lpfTemp = input.temp[0];

        for (int i = 0; i < 3; i++)
        {
            tmpScomp.acc[0].lpfLsb[i] = input.imu3.I[0].acc[i];
            tmpScomp.pqr[0].lpfLsb[i] = input.imu3.I[0].pqr[i];
        }
    }
    else
    {	// Low-pass filter sensor data
        O0_LP_FILTER_local(tmpScomp.acc[0].lpfTemp, input.temp[0], ALPH_750SR_0p01CF, BETA_750SR_0p01CF);
        O0_LP_FILTER_local(tmpScomp.pqr[0].lpfTemp, input.temp[0], ALPH_750SR_0p01CF, BETA_750SR_0p01CF);

        for (int i = 0; i < 3; i++)
        {
            O0_LP_FILTER_local(tmpScomp.pqr[0].lpfLsb[i], input.imu3.I[0].pqr[i], ALPH_750SR_0p01CF, BETA_750SR_0p01CF);
            O0_LP_FILTER_local(tmpScomp.acc[0].lpfLsb[i], input.imu3.I[0].acc[i], ALPH_750SR_0p01CF, BETA_750SR_0p01CF);
        }
    }



    // Run Low Pass Filter
    //step_lpf_sens_comp(tmpScomp.pqr[0], ALPH_750SR_0p001CF, BETA_750SR_0p001CF, input.temp[0], input.imu3.I[0].pqr);
    //step_lpf_sens_comp(tmpScomp.acc[0], ALPH_750SR_0p001CF, BETA_750SR_0p001CF, input.temp[0], input.imu3.I[0].acc);
}

/*void sendCurrentScomp(int i)
{
    memset(&tmpScomp, 0, sizeof(sensor_compensation_t));

    // set time
    tmpScomp.timeMs = i * DELTA_TIME_MS;

    // C
    tmpScomp.acc[_X_].lpfTemp = parsedData[i].temperature;
    tmpScomp.pqr[_X_].lpfTemp = parsedData[i].temperature;

    // M/s^2
    tmpScomp.acc[0].lpfLsb[_X_] = parsedData[i].accX;
    tmpScomp.acc[0].lpfLsb[_Y_] = parsedData[i].accY;
    tmpScomp.acc[0].lpfLsb[_Z_] = parsedData[i].accZ;

    // rad/s
    tmpScomp.pqr[0].lpfLsb[_X_] = parsedData[i].gyX;
    tmpScomp.pqr[0].lpfLsb[_Y_] = parsedData[i].gyY;
    tmpScomp.pqr[0].lpfLsb[_Z_] = parsedData[i].gyZ;


    logger.LogData(devLog, &scompHdr, (const uint8_t*)&tmpScomp);
}*/

void setCurrentPimu(int i) // target 5.236 / 80 radians per period. .06545
{
    double sumThetaX = 0;
    double sumThetaY = 0;
    double sumThetaZ = 0;
    double degPerSX = 0;
    double degPerSY = 0;
    double degPerSZ = 0;

    double sumVelX = 0;
    double sumVelY = 0;
    double sumVelZ = 0;
    double gAccX = 0;
    double gAccY = 0;
    double gAccZ = 0;
    
    tmpPimu.dt = DELTA_TIME_MS * 10;
    tmpPimu.time = i * DELTA_TIME_MS;

    static int lastIndex = 0;

    // sum previous samples
    for (int j = i - 9; j <= i; j++)
    {
        if ((lastIndex+1) != parsedData[j].index)
        {
            printf("last: %d, current: %d\r\n", lastIndex, parsedData[j].index);
        }
        lastIndex = parsedData[j].index;

        sumThetaX += parsedData[j].gyX;
        sumThetaY += parsedData[j].gyY;
        sumThetaZ += parsedData[j].gyZ;

        sumVelX += parsedData[j].accX;
        sumVelY += parsedData[j].accY;
        sumVelZ += parsedData[j].accZ;
    }

    degPerSX = (sumThetaX / 10.0) * GYRO_SCALE;
    degPerSY = (sumThetaY / 10.0) * GYRO_SCALE;
    degPerSZ = (sumThetaZ / 10.0) * GYRO_SCALE;

    // ave accel over period in g
    gAccX = (sumVelX / 10.0) * .00048828125;
    gAccY = (sumVelY / 10.0) * .00048828125;
    gAccZ = (sumVelZ / 10.0) * .00048828125;

    tmpPimu.theta[_X_] = degPerSX * C_DEG2RAD * tmpPimu.dt;
    tmpPimu.theta[_Y_] = degPerSY * C_DEG2RAD * tmpPimu.dt;
    tmpPimu.theta[_Z_] = degPerSZ * C_DEG2RAD * tmpPimu.dt;

    // Convert to m/s = g * G * dt
    tmpPimu.vel[_X_] = gAccX * tmpPimu.dt * 9.80665;
    tmpPimu.vel[_Y_] = gAccY * tmpPimu.dt * 9.80665;
    tmpPimu.vel[_Z_] = gAccZ * tmpPimu.dt * 9.80665;
}

int writeOutputFile()
{
    cISLogger logger;

    p_data_hdr_t pimuHdr;
    p_data_hdr_t scompHdr;
    p_data_hdr_t devInfoHdr;

    cISLogger::sSaveOptions options; // = cISLogger::sSaveOptions();

    options.logType = cISLogger::eLogType::LOGTYPE_DAT;
    options.driveUsageLimitPercent = 99;
    
    logger.InitSave(outFileName, options);
    auto devLog = logger.registerDevice(IS_HARDWARE_TYPE_IMX, 12345);   // if you know this information, you can pass it, but it's not important that it match your actual hardware.
    logger.EnableLogging(true);

    setStructDefaults();

    pimuHdr.id = DID_PIMU;
    pimuHdr.size = sizeof(pimu_t);
    pimuHdr.offset = 0;

    scompHdr.id = DID_SCOMP;
    scompHdr.size = sizeof(sensor_compensation_t);
    scompHdr.offset = 0;

    devInfoHdr.id = DID_DEV_INFO;
    devInfoHdr.size = sizeof(dev_info_t);
    devInfoHdr.offset = 0;

    // send first dev info
    logger.LogData(devLog, &devInfoHdr, (const uint8_t*)&tmpDevInfo);

    for (int i = 0; i < parseCnt; i++)
    {

        if (i == 800 * 15)
            cout << "asdf";

        // run LPF
        runScompLpf(i);

        // log scomp
        if ((i > 48000) && ((i % 800) == 0))
        {
            // set time
            tmpScomp.timeMs = parsedData[i].timeMs;
            logger.LogData(devLog, &scompHdr, (const uint8_t*)&tmpScomp);
        }

        // log pimu
        if ((i % 10) == 0 && (i > 9))
        {
            setCurrentPimu(i);
            //cout << "time: " << tmpPimu.time << "s.\r\n";
            logger.LogData(devLog, &pimuHdr, (const uint8_t*)&tmpPimu);
        }

        // log dev info 1 time per second
        if ((i % BROAD_CAST_RATE) == (BROAD_CAST_RATE/2))
            logger.LogData(devLog, &devInfoHdr, (const uint8_t*)&tmpDevInfo);

        if (i % 100000 == 1)
            cout << "Writing line: " << i << ".\r\n";
    }

    logger.CloseAllFiles();

    return 0;
}

void setStructDefaults()
{
    tmpDevInfo.buildYear = 24;
    tmpDevInfo.buildMonth = 3;
    tmpDevInfo.buildDay = 11;

    tmpDevInfo.serialNumber = 123456;

    tmpDevInfo.hardwareVer[0] = 1;
    tmpDevInfo.hardwareVer[1] = 2;
    tmpDevInfo.hardwareVer[2] = 3;
    tmpDevInfo.hardwareVer[3] = 4;

    tmpDevInfo.firmwareVer[0] = 1;
    tmpDevInfo.firmwareVer[1] = 2;
    tmpDevInfo.firmwareVer[2] = 3;
    tmpDevInfo.firmwareVer[3] = 4;

    tmpDevInfo.buildNumber = 123456;

    tmpDevInfo.protocolVer[0] = 1;
    tmpDevInfo.protocolVer[1] = 2;
    tmpDevInfo.protocolVer[2] = 3;
    tmpDevInfo.protocolVer[3] = 4;

    tmpDevInfo.repoRevision = 159;

    tmpDevInfo.buildType = 'r';

    tmpDevInfo.buildHour = 16;
    tmpDevInfo.buildMinute = 8;
    tmpDevInfo.buildSecond = 4;
    tmpDevInfo.buildMillisecond = 123;

    //tmpScomp.calState = 6;

    tmpPimu.dt = 10 * DELTA_TIME_MS;
    tmpPimu.status = IMU_STATUS_IMU_OK_MASK;
}

int main(int argc, char* argv[])
{
    int err = -1;

    if (argc == 3)
    {
        inFileName = argv[1];
        outFileName = argv[2];

        err = parseInputFile();

        if (err) return err;

        err = writeOutputFile();
    }
    else
        cout << "Please input and output file!\n";


    cout << "Done: " << err << "!\n";

    return err;
}

// Run program: Ctrl + F5 or Debug > Start Without Debugging menu
// Debug program: F5 or Debug > Start Debugging menu

// Tips for Getting Started: 
//   1. Use the Solution Explorer window to add/manage files
//   2. Use the Team Explorer window to connect to source control
//   3. Use the Output window to see build output and other messages
//   4. Use the Error List window to view errors
//   5. Go to Project > Add New Item to create new code files, or Project > Add Existing Item to add existing code files to the project
//   6. In the future, to open this project again, go to File > Open > Project and select the .sln file
