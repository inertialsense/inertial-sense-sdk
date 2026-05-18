#include <string.h>

#include "IS_calibration_convert.h"
#include "data_sets.h"

#ifndef _MIN
#define _MIN(a,b) (((a)<(b))?(a):(b))
#endif

void set_sensor_mcal_data_defaults_v1p4(sensor_cal_v1p4_data_t *data)
{
    if (data == NULL) { return; }

    memset(&data->mcal, 0, sizeof(sensor_mcal_group_v1p4_t));
    for (int d = 0; d < NUM_IMU_DEVICES_V1P4; d++)
    {
        data->mcal.pqr[d].orth[0] = 1;
        data->mcal.pqr[d].orth[4] = 1;
        data->mcal.pqr[d].orth[8] = 1;

        data->mcal.acc[d].orth[0] = 1;
        data->mcal.acc[d].orth[4] = 1;
        data->mcal.acc[d].orth[8] = 1;
    }
    for (int d = 0; d < NUM_MAG_DEVICES_V1P4; d++)
    {
        data->mcal.mag[d].orth[0] = 1;
        data->mcal.mag[d].orth[4] = 1;
        data->mcal.mag[d].orth[8] = 1;
    }

    // Set data info (size and checksum)
    data->dinfo.size = sizeof(sensor_cal_v1p4_data_t);
    data->dinfo.checksum = flashChecksum32(data, data->dinfo.size);
}

void set_sensor_mcal_data_defaults_v1p5(sensor_cal_v1p5_data_t *data)
{
    if (data == NULL) { return; }

    memset(&data->mcal, 0, sizeof(sensor_mcal_group_v1p5_t));
    for (int d = 0; d < NUM_IMU_DEVICES_V1P5; d++)
    {
        data->mcal.pqr[d].orth[0] = 1;
        data->mcal.pqr[d].orth[4] = 1;
        data->mcal.pqr[d].orth[8] = 1;

        data->mcal.acc[d].orth[0] = 1;
        data->mcal.acc[d].orth[4] = 1;
        data->mcal.acc[d].orth[8] = 1;
    }

    // Set data info (size and checksum)
    data->dinfo.size = sizeof(sensor_cal_v1p5_data_t);
    data->dinfo.checksum = flashChecksum32(data, data->dinfo.size);
}

void set_sensor_cal_data_defaults_v1p4(sensor_cal_v1p4_data_t *data)
{
    if (data == NULL) { return; }

    memset(data, 0, sizeof(sensor_cal_v1p4_data_t));    // Default tcal
    set_sensor_mcal_data_defaults_v1p4(data);           // Default mcal and set size and checksum for data struct
}

void set_sensor_cal_data_defaults_v1p5(sensor_cal_v1p5_data_t *data)
{
    if (data == NULL) { return; }

    memset(data, 0, sizeof(sensor_cal_v1p5_data_t));    // Default tcal
    set_sensor_mcal_data_defaults_v1p5(data);           // Default mcal and set size and checksum for data struct
}

void set_sensor_mcal_data_defaults_v1p3(sensor_cal_v1p3_data_t *data)
{
    if (data == NULL) { return; }

    memset(&data->mcal, 0, sizeof(sensor_mcal_group_v1p3_t));    // Default mcal
    for (int d = 0; d < NUM_IMU_DEVICES_V1P3; d++)
    {
        data->mcal.pqr[d].orth[0] = 1;
        data->mcal.pqr[d].orth[4] = 1;
        data->mcal.pqr[d].orth[8] = 1;

        data->mcal.acc[d].orth[0] = 1;
        data->mcal.acc[d].orth[4] = 1;
        data->mcal.acc[d].orth[8] = 1;
    }
    for (int d = 0; d < NUM_MAG_DEVICES_V1P3; d++)
    {
        data->mcal.mag[d].orth[0] = 1;
        data->mcal.mag[d].orth[4] = 1;
        data->mcal.mag[d].orth[8] = 1;
    }

    // Set data info (size and checksum)
    data->dinfo.size = sizeof(sensor_cal_v1p3_data_t);
    data->dinfo.checksum = flashChecksum32(data, data->dinfo.size);
}

void set_sensor_cal_data_defaults_v1p3(sensor_cal_v1p3_data_t *data)
{
    if (data == NULL) { return; }

    memset(data, 0, sizeof(sensor_cal_v1p3_data_t));    // Default tcal
    set_sensor_mcal_data_defaults_v1p3(data);           // Default mcal and set size and checksum for data struct
}

void convert_tcal_v1p3_to_v1p4(const sensor_tcal_group_v1p3_t *v1p3, sensor_tcal_group_v1p4_t *v1p4)
{
    for (int d = 0; d < _MIN(NUM_IMU_DEVICES_V1P3, NUM_IMU_DEVICES_V1P4); d++)
    {
        v1p4->gyr[d] = v1p3->gyr[d];
        v1p4->acc[d] = v1p3->acc[d];
    }
    for (int d = 0; d < _MIN(NUM_MAG_DEVICES_V1P3, NUM_MAG_DEVICES_V1P4); d++)
    {
        v1p4->mag[d] = v1p3->mag[d];
    }
}

void convert_mcal_v1p3_to_v1p4(const sensor_mcal_group_v1p3_t *v1p3, sensor_mcal_group_v1p4_t *v1p4)
{
    for (int d = 0; d < _MIN(NUM_IMU_DEVICES_V1P3, NUM_IMU_DEVICES_V1P4); d++)
    {
        v1p4->pqr[d] = v1p3->pqr[d];
        v1p4->acc[d] = v1p3->acc[d];
    }
    for (int d = 0; d < _MIN(NUM_MAG_DEVICES_V1P3, NUM_MAG_DEVICES_V1P4); d++)
    {
        v1p4->mag[d] = v1p3->mag[d];
    }
}

void convert_sensor_cal_v1p3_to_v1p4(const sensor_cal_v1p3_t *v1p3, sensor_cal_v1p4_t *v1p4)
{
    v1p4->info = v1p3->info;
    v1p4->info.version[0] = 1;
    v1p4->info.version[1] = 4;
    v1p4->info.version[2] = 0;
    v1p4->info.version[3] = 0;
    v1p4->info.size = sizeof(sensor_cal_info_t);
    v1p4->info.checksum = flashChecksum32(&v1p4->info, v1p4->info.size);

    set_sensor_cal_data_defaults_v1p4(&v1p4->data);
    convert_tcal_v1p3_to_v1p4(&(v1p3->data.tcal), &(v1p4->data.tcal));
    convert_mcal_v1p3_to_v1p4(&(v1p3->data.mcal), &(v1p4->data.mcal));
    v1p4->data.dinfo.size = sizeof(sensor_cal_v1p4_data_t);
    v1p4->data.dinfo.checksum = flashChecksum32(&v1p4->data, v1p4->data.dinfo.size);
}

void convert_sensor_cal_v1p4_to_v1p5(const sensor_cal_v1p4_t *v1p4, sensor_cal_v1p5_t *v1p5)
{
    // For v1.5, we are removing magnetometer motion cal, so just update the data size and checksum.

    v1p5->data.dinfo.size = sizeof(sensor_cal_v1p5_data_t);
    v1p5->data.dinfo.checksum = flashChecksum32(&v1p5->data, v1p5->data.dinfo.size);
}

void convert_tcal_v1p4_to_v1p3(const sensor_tcal_group_v1p4_t *v1p4, sensor_tcal_group_v1p3_t *v1p3)
{
    for (int d = 0; d < _MIN(NUM_IMU_DEVICES_V1P3, NUM_IMU_DEVICES_V1P4); d++)
    {
        v1p3->gyr[d] = v1p4->gyr[d];
        v1p3->acc[d] = v1p4->acc[d];
    }
    for (int d = 0; d < _MIN(NUM_MAG_DEVICES_V1P3, NUM_MAG_DEVICES_V1P4); d++)
    {
        v1p3->mag[d] = v1p4->mag[d];
    }
}

void convert_mcal_v1p4_to_v1p3(const sensor_mcal_group_v1p4_t *v1p4, sensor_mcal_group_v1p3_t *v1p3)
{
    for (int d = 0; d < _MIN(NUM_IMU_DEVICES_V1P3, NUM_IMU_DEVICES_V1P4); d++)
    {
        v1p3->pqr[d] = v1p4->pqr[d];
        v1p3->acc[d] = v1p4->acc[d];
    }
    for (int d = 0; d < _MIN(NUM_MAG_DEVICES_V1P3, NUM_MAG_DEVICES_V1P4); d++)
    {
        v1p3->mag[d] = v1p4->mag[d];
    }
}

void convert_sensor_cal_v1p4_to_v1p3(const sensor_cal_v1p4_t *v1p4, sensor_cal_v1p3_t *v1p3)
{
    v1p3->info = v1p4->info;
    v1p3->info.version[0] = 1;
    v1p3->info.version[1] = 3;
    v1p3->info.version[2] = 0;
    v1p3->info.version[3] = 0;
    v1p3->info.size = sizeof(sensor_cal_info_t);
    v1p3->info.checksum = flashChecksum32(&v1p3->info, v1p3->info.size);

    set_sensor_cal_data_defaults_v1p3(&(v1p3->data));
    convert_tcal_v1p4_to_v1p3(&(v1p4->data.tcal), &(v1p3->data.tcal));
    convert_mcal_v1p4_to_v1p3(&(v1p4->data.mcal), &(v1p3->data.mcal));
    v1p3->data.dinfo.size = sizeof(sensor_cal_v1p3_data_t);
    v1p3->data.dinfo.checksum = flashChecksum32(&v1p3->data, v1p3->data.dinfo.size);
}
