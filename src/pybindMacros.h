// support types
PYBIND11_NUMPY_DTYPE(gps_sat_sv_t, gnssId, svId, cno, elev, azim, prRes, flags); 
PYBIND11_NUMPY_DTYPE(sensors_mpu_w_temp_t, pqr, acc, mag, temp);
PYBIND11_NUMPY_DTYPE(rtos_task_t, name, priority, stackUnused, periodMs, runTimeUs, maxRunTimeUs, averageRunTimeUs, gapCount, cpuUsage, handle);
PYBIND11_NUMPY_DTYPE(imus_t, pqr, acc);
PYBIND11_NUMPY_DTYPE(sensors_mpu_t, pqr, acc, mag);



// Public Typcs
PYBIND11_NUMPY_DTYPE(dev_info_t, reserved, serialNumber, hardwareVer, firmwareVer, buildNumber, protocolVer, repoRevision, manufacturer, buildDate, buildTime, addInfo);
PYBIND11_NUMPY_DTYPE(crash_info_t, r0, r1, r2, r3, r12, lr, pc, psr);
PYBIND11_NUMPY_DTYPE(preintegrated_imu_t, time, theta1, theta2, vel1, vel2, dt);
PYBIND11_NUMPY_DTYPE(ins_1_t, week, timeOfWeek, insStatus, hdwStatus, theta, uvw, lla, ned);
PYBIND11_NUMPY_DTYPE(ins_2_t, week, timeOfWeek, insStatus, hdwStatus, qn2b, uvw, lla);
PYBIND11_NUMPY_DTYPE(gps_pos_t, week, timeOfWeekMs, status, ecef, lla, hMSL, hAcc, vAcc, pDop, cnoMean, towOffset);
PYBIND11_NUMPY_DTYPE(config_t, system, invSystem);
PYBIND11_NUMPY_DTYPE(ascii_msgs_t, options, pimu, ppimu, pins1, pins2, pgpsp, reserved, gpgga, gpgll, gpgsa, gprmc);
PYBIND11_NUMPY_DTYPE(rmc_t, bits, options);
PYBIND11_NUMPY_DTYPE(sys_params_t, timeOfWeekMs, insStatus, hdwStatus, imuTemp, baroTemp, mcuTemp, reserved1, imuPeriodMs, navPeriodMs, reserved2, genFaultCode);
PYBIND11_NUMPY_DTYPE(sys_sensors_t, time, temp, pqr, acc, mag, bar, barTemp, mslBar, humidity, vin, ana1, ana3, ana4);
PYBIND11_NUMPY_DTYPE(nvm_flash_cfg_t, size, checksum, key, startupImuDtMs, startupNavDtMs, ser0BaudRate, ser1BaudRate, insRotation, insOffset, gps1AntOffset, insDynModel, sysCfgBits, refLla, lastLla, lastLlaTimeOfWeekMs, lastLlaWeek, lastLlaUpdateDistance, ioConfig, cBrdConfig, gps2AntOffset, zeroVelRotation, zeroVelOffset, magInclination, magDeclination, gpsTimeSyncPeriodMs, startupGPSDtMs, RTKCfgBits, sensorConfig);
PYBIND11_NUMPY_DTYPE(gps_sat_t, timeOfWeekMs, numSats, sat);
PYBIND11_NUMPY_DTYPE(gps_version_t, swVersion, hwVersion, extension, reserved);
PYBIND11_NUMPY_DTYPE(mag_cal_t, enMagRecal, progress, declination);
PYBIND11_NUMPY_DTYPE(internal_diagnostic_t, gapCountSerialDriver, gapCountSerialParser, rxOverflowCount, txOverflowCount, checksumFailCount);
PYBIND11_NUMPY_DTYPE(gps_rtk_rel_t, timeOfWeekMs, differentialAge, arRatio, vectorToBase, distanceToBase, headingToBase);
PYBIND11_NUMPY_DTYPE(gps_rtk_misc_t, timeOfWeekMs, accuracyPos, accuracyCov, arThreshold, gDop, hDop, vDop, baseLla, cycleSlipCount, roverGpsObservationCount, baseGpsObservationCount, roverGlonassObservationCount, baseGlonassObservationCount, roverGalileoObservationCount, baseGalileoObservationCount, roverBeidouObservationCount, baseBeidouObservationCount, roverQzsObservationCount, baseQzsObservationCount, roverGpsEphemerisCount, baseGpsEphemerisCount, roverGlonassEphemerisCount, baseGlonassEphemerisCount, roverGalileoEphemerisCount, baseGalileoEphemerisCount, roverBeidouEphemerisCount, baseBeidouEphemerisCount, roverQzsEphemerisCount, baseQzsEphemerisCount, roverSbasCount, baseSbasCount, baseAntennaCount, ionUtcAlmCount);
PYBIND11_NUMPY_DTYPE(sensors_t, mpu);
PYBIND11_NUMPY_DTYPE(io_t, timeOfWeekMs, gpioStatus);
PYBIND11_NUMPY_DTYPE(sys_sensors_adc_t, time, mpu, bar, barTemp, humidity, ana);
PYBIND11_NUMPY_DTYPE(gps_vel_t, timeOfWeekMs, velEcef, sAcc);
PYBIND11_NUMPY_DTYPE(rtos_info_t, task, freeHeapSize, mallocMinusFree);
PYBIND11_NUMPY_DTYPE(inl2_states_t, timeOfWeek, qe2b, ve, ecef, biasPqr, biasAcc, biasBaro, magDec, magInc);
PYBIND11_NUMPY_DTYPE(magnetometer_t, time, mag);
PYBIND11_NUMPY_DTYPE(barometer_t, time, bar, mslBar, barTemp, humidity);
PYBIND11_NUMPY_DTYPE(dual_imu_t, time, I);
//PYBIND11_NUMPY_DTYPE(gps_raw_t, receiverIndex, dataType, obsCount, reserved, data);
//PYBIND11_NUMPY_DTYPE(gps_rtk_opt_t, mode, soltype, nf, navsys, elmin, snrmin, modear, glomodear, gpsmodear, bdsmodear, arfilter, maxout, minlock, minfixsats, minholdsats, mindropsats, rcvstds, minfix, armaxiter, dynamics, niter, intpref, rovpos, refpos, eratio, err, std, prn, sclkstab, thresar, elmaskar, elmaskhold, thresslip, varholdamb, gainholdamb, maxtdiff, maxinno, maxrejc, maxgdop, baseline, ru, rb, maxaveep, outsingle, prcopt_t);
PYBIND11_NUMPY_DTYPE(manufacturing_info_t, serialNumber, lotNumber, date, key);
PYBIND11_NUMPY_DTYPE(bit_t, state, hdwBitStatus, calBitStatus, tcPqrBias, tcAccBias, tcPqrSlope, tcAccSlope, tcPqrLinearity, tcAccLinearity, pqr, acc, pqrSigma, accSigma);
PYBIND11_NUMPY_DTYPE(ins_3_t, week, timeOfWeek, insStatus, hdwStatus, qn2b, uvw, lla, msl);
PYBIND11_NUMPY_DTYPE(ins_4_t, week, timeOfWeek, insStatus, hdwStatus, qe2b, ve, ecef);
PYBIND11_NUMPY_DTYPE(inl2_ned_sigma_t, timeOfWeekMs, PxyzNED, PvelNED, PattNED, PABias, PWBias, PBaroBias, PDeclination);
PYBIND11_NUMPY_DTYPE(strobe_in_time_t, week, timeOfWeekMs, pin, count);
PYBIND11_NUMPY_DTYPE(velocity_sensor_t, time_ms, id, vel, cov, q, p, valid, reserved);
PYBIND11_NUMPY_DTYPE(diag_msg_t, timeOfWeekMs, messageLength, message);
PYBIND11_NUMPY_DTYPE(survey_in_t, state, maxDurationSec, minAccuracy, elapsedTimeSec, hAccuracy, lla);
PYBIND11_NUMPY_DTYPE(evb2_t, firmwareVer, comBridgeCfg, loggerState, loggerElapsedTimeMs, ipAddr);
//PYBIND11_NUMPY_DTYPE(port_monitor_t, portNumber, txTimeMs, txBytesPerS, rxTimeMs, rxBytesPerS, status);


// Internal Data types
PYBIND11_NUMPY_DTYPE(feature_bits_t, key, featureBits, hash1, hash2);
PYBIND11_NUMPY_DTYPE(sensor_comp_unit_t, lpfLsb, temp, tempRampRate, tci, numTcPts, dtTemp);
PYBIND11_NUMPY_DTYPE(imu1_t, pqr, acc, mag);
PYBIND11_NUMPY_DTYPE(sensors_w_temp_t, mpu);
PYBIND11_NUMPY_DTYPE(sensor_bias_t, timeOfWeekMs, pqr, acc, mslBar, magI, magB);
PYBIND11_NUMPY_DTYPE(sensor_compensation_t, mpu, sampleCount, calState, alignAccel, status);
PYBIND11_NUMPY_DTYPE(hdw_params_t, timeOfWeekMs, pqrDev, accDev, pqrSigma, accSigma, mean, update, gpsCnoSigma, gpsCnoMean);
PYBIND11_NUMPY_DTYPE(nvr_manage_t, flash_write_needed, flash_write_count);
PYBIND11_NUMPY_DTYPE(debug_string_t, s);
PYBIND11_NUMPY_DTYPE(debug_array_t, i, f, lf);
PYBIND11_NUMPY_DTYPE(ins_dev_1_t, week, timeOfWeek, insStatus, hdwStatus, euler, uvw, lla, ned, eulerErr, uvwErr, nedErr);
PYBIND11_NUMPY_DTYPE(inl2_status_t, ahrs, zero_accel, zero_angrate, accel_motion, rot_motion, zero_vel, ahrs_gps_cnt, att_err, att_coarse, att_aligned, att_aligning, start_proc_done, mag_cal_good, mag_cal_done, stat_magfield);
PYBIND11_NUMPY_DTYPE(inl2_misc_t, gps_time_last_valid);
