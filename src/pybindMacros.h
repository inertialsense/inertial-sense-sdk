#include "data_sets.h"

// support types
PYBIND11_NUMPY_DTYPE(gps_sat_sv_t, gnssId, svId, elev, azim, cno, status); 
PYBIND11_NUMPY_DTYPE(gps_sig_sv_t, gnssId, svId, sigId, cno, quality, status); 
PYBIND11_NUMPY_DTYPE(sensors_imu_w_temp_t, pqr, acc, temp);
PYBIND11_NUMPY_DTYPE(sensors_mag_t, mag);
PYBIND11_NUMPY_DTYPE(rtos_task_t, name, priority, stackUnused, periodMs, runtimeUs, avgRuntimeUs, lowerRuntimeUs, upperRuntimeUs, maxRuntimeUs, startTimeUs, gapCount, doubleGapCount, reserved, cpuUsage, handle);
PYBIND11_NUMPY_DTYPE(imus_t, pqr, acc);
PYBIND11_NUMPY_DTYPE(imu3_t, time, status, I);
PYBIND11_NUMPY_DTYPE(mag_xyz_t, xyz);
PYBIND11_NUMPY_DTYPE(sensors_mpu_t, pqr, acc, mag);
PYBIND11_NUMPY_DTYPE(wheel_encoder_t, timeOfWeek, status, theta_l, theta_r, omega_l, omega_r, wrap_count_l, wrap_count_r);
PYBIND11_NUMPY_DTYPE(wheel_transform_t, e_b2w, e_b2w_sigma, t_b2w, t_b2w_sigma);
PYBIND11_NUMPY_DTYPE(wheel_config_t, bits, transform, track_width, radius);
PYBIND11_NUMPY_DTYPE(ground_vehicle_t, timeOfWeekMs, status, mode, wheelConfig);
PYBIND11_NUMPY_DTYPE(evb_luna_velocity_control_vehicle_t, velCmd_f, velCmd_w, velCmdMnl_f, velCmdMnl_w, velCmdSlew_f, velCmdSlew_w, vel_f, vel_w, err_f, err_w, eff_f, eff_w);
PYBIND11_NUMPY_DTYPE(evb_luna_velocity_control_wheel_t, velCmd, velCmdSlew, vel, err, ff_eff, fb_eff, fb_eff_integral, eff, effInt, effDuty);
PYBIND11_NUMPY_DTYPE(evb_luna_velocity_control_t, timeMs, dt, current_mode, status, vehicle, wheel_l, wheel_r, potV_l, potV_r);
PYBIND11_NUMPY_DTYPE(nmeaBroadcastMsgPair_t, msgID, msgPeriod);

// Public Types
PYBIND11_NUMPY_DTYPE(dev_info_t, reserved, serialNumber, hardwareVer, firmwareVer, buildNumber, protocolVer, repoRevision, manufacturer, buildType, buildYear, buildMonth, buildDay, buildHour, buildMinute, buildSecond, buildMillisecond, addInfo);
PYBIND11_NUMPY_DTYPE(system_fault_t, status, g1Task, g2FileNum, g3LineNum, g4, g5Lr, pc, psr);
PYBIND11_NUMPY_DTYPE(pimu_t, time, dt, status, theta, vel);
PYBIND11_NUMPY_DTYPE(ins_1_t, week, timeOfWeek, insStatus, hdwStatus, theta, uvw, lla, ned);
PYBIND11_NUMPY_DTYPE(ins_2_t, week, timeOfWeek, insStatus, hdwStatus, qn2b, uvw, lla);
PYBIND11_NUMPY_DTYPE(ins_3_t, week, timeOfWeek, insStatus, hdwStatus, qn2b, uvw, lla, msl);
PYBIND11_NUMPY_DTYPE(ins_4_t, week, timeOfWeek, insStatus, hdwStatus, qe2b, ve, ecef);
PYBIND11_NUMPY_DTYPE(system_command_t, command, invCommand);
PYBIND11_NUMPY_DTYPE(nmea_msgs_t, options, nmeaBroadcastMsgs);
PYBIND11_NUMPY_DTYPE(rmc_t, bits, options);
PYBIND11_NUMPY_DTYPE(sys_params_t, timeOfWeekMs, insStatus, hdwStatus, imuTemp, baroTemp, mcuTemp, sysStatus, imuSamplePeriodMs, navOutputPeriodMs, sensorTruePeriod, flashCfgChecksum, navUpdatePeriodMs, genFaultCode, upTime);
PYBIND11_NUMPY_DTYPE(sys_sensors_t, time, temp, pqr, acc, mag, bar, barTemp, mslBar, humidity, vin, ana1, ana3, ana4);
PYBIND11_NUMPY_DTYPE(nvm_flash_cfg_t, size, checksum, key, startupImuDtMs, startupNavDtMs, ser0BaudRate, ser1BaudRate, insRotation, insOffset, gps1AntOffset, dynamicModel, debug, gnssSatSigConst, sysCfgBits, refLla, lastLla, lastLlaTimeOfWeekMs, lastLlaWeek, lastLlaUpdateDistance, ioConfig, platformConfig, gps2AntOffset, zeroVelRotation, zeroVelOffset, gpsTimeUserDelay, magDeclination, gpsTimeSyncPeriodMs, startupGPSDtMs, RTKCfgBits, sensorConfig, gpsMinimumElevation, ser2BaudRate, wheelConfig, magInterferenceThreshold, magCalibrationQualityThreshold, gnssCn0Minimum, gnssCn0DynMinOffset, imuRejectThreshGyroLow, imuRejectThreshGyroHigh, imuShockDetectLatencyMs, imuShockRejectLatchMs, imuShockOptions, imuShockDeltaAccHighThreshold, imuShockDeltaAccLowThreshold, imuShockDeltaGyroHighThreshold, imuShockDeltaGyroLowThreshold, ioConfig2);
PYBIND11_NUMPY_DTYPE(gps_pos_t, week, timeOfWeekMs, status, ecef, lla, hMSL, hAcc, vAcc, pDop, cnoMean, towOffset, leapS, satsUsed, cnoMeanSigma, status2);
PYBIND11_NUMPY_DTYPE(gps_vel_t, timeOfWeekMs, vel, sAcc, status);
PYBIND11_NUMPY_DTYPE(gps_sat_t, timeOfWeekMs, numSats, sat);
PYBIND11_NUMPY_DTYPE(gps_sig_t, timeOfWeekMs, numSigs, sig);
PYBIND11_NUMPY_DTYPE(gps_version_t, swVersion, hwVersion, extension);
PYBIND11_NUMPY_DTYPE(mag_cal_t, state, progress, declination);
PYBIND11_NUMPY_DTYPE(gps_rtk_rel_t, timeOfWeekMs, differentialAge, arRatio, baseToRoverVector, baseToRoverDistance, baseToRoverHeading, baseToRoverHeadingAcc, status);
PYBIND11_NUMPY_DTYPE(gps_rtk_misc_t, timeOfWeekMs, accuracyPos, accuracyCov, arThreshold, gDop, hDop, vDop, baseLla, cycleSlipCount, roverGpsObservationCount, baseGpsObservationCount, roverGlonassObservationCount, baseGlonassObservationCount, roverGalileoObservationCount, baseGalileoObservationCount, roverBeidouObservationCount, baseBeidouObservationCount, roverQzsObservationCount, baseQzsObservationCount, roverGpsEphemerisCount, baseGpsEphemerisCount, roverGlonassEphemerisCount, baseGlonassEphemerisCount, roverGalileoEphemerisCount, baseGalileoEphemerisCount, roverBeidouEphemerisCount, baseBeidouEphemerisCount, roverQzsEphemerisCount, baseQzsEphemerisCount, roverSbasCount, baseSbasCount, baseAntennaCount, ionUtcAlmCount, correctionChecksumFailures, timeToFirstFixMs);
// PYBIND11_NUMPY_DTYPE(sensors_t, time, temp, pqr, acc, mag, bar, barTemp, mslBar, humidity, vin, ana1, ana3, ana4);

PYBIND11_NUMPY_DTYPE(sensors_t, time, mpu);
PYBIND11_NUMPY_DTYPE(sensor_comp_unit_t, lpfLsb, lpfTemp, k, temp, tempRampRate, tci, numTcPts, dtTemp);
PYBIND11_NUMPY_DTYPE(sensor_compensation_t, timeMs, pqr, acc, mag, referenceImu, referenceMag, sampleCount, calState, status, alignAccel);
PYBIND11_NUMPY_DTYPE(sensors_w_temp_t, imu3, temp, mag);

PYBIND11_NUMPY_DTYPE(sys_sensors_adc_t, time, imu, mag, bar, barTemp, humidity, ana);
PYBIND11_NUMPY_DTYPE(rtos_info_t, freeHeapSize, mallocSize, freeSize, task);
PYBIND11_NUMPY_DTYPE(inl2_states_t, timeOfWeek, qe2b, ve, ecef, biasPqr, biasAcc, biasBaro, magDec, magInc);
PYBIND11_NUMPY_DTYPE(inl2_status_t, ahrs, zero_accel, zero_angrate, accel_motion, rot_motion, zero_vel, ahrs_gps_cnt, hdg_err, hdg_coarse, hdg_aligned, hdg_aligning, ekf_init_done, mag_cal_good, mag_cal_done, stat_magfield);
PYBIND11_NUMPY_DTYPE(magnetometer_t, time, mag);
PYBIND11_NUMPY_DTYPE(barometer_t, time, bar, mslBar, barTemp, humidity);
PYBIND11_NUMPY_DTYPE(imu_t, time, status, I);	
PYBIND11_NUMPY_DTYPE(inl2_mag_obs_info_t, timeOfWeekMs, Ncal_samples, ready, calibrated, auto_recal, outlier, magHdg, insHdg, magInsHdgDelta, nis, nis_threshold, Wcal, activeCalSet, magHdgOffset, Tcal, bias_cal);
// PYBIND11_NUMPY_DTYPE(gps_raw_t, receiverIndex, dataType, obsCount, reserved, data);
// PYBIND11_NUMPY_DTYPE(gps_rtk_opt_t, mode, soltype, nf, navsys, elmin, snrmin, modear, glomodear, gpsmodear, sbsmodear, bdsmodear, arfilter, maxout, maxrej, minlock, minfixsats, minholdsats, mindropsats, rcvstds, minfix, armaxiter, dynamics, niter, intpref, rovpos, refpos, eratio, err, std, prn, sclkstab, thresar, elmaskar, elmaskhold, thresslip, varholdamb, gainholdamb, maxtdiff, fix_reset_base_msgs, maxinnocode, maxinnophase, maxnis, maxgdop, baseline, max_baseline_error, reset_baseline_error, max_ubx_error, ru, rb, maxaveep, outsingle, prcopt_t);
PYBIND11_NUMPY_DTYPE(manufacturing_info_t, serialNumber, lotNumber, date, key, uid);
PYBIND11_NUMPY_DTYPE(bit_t, command, lastCommand, state, reserved, hdwBitStatus, calBitStatus, tcPqrBias, tcAccBias, tcPqrSlope, tcAccSlope, tcPqrLinearity, tcAccLinearity, pqr, acc, pqrSigma, accSigma, testMode, testVar, detectedHardwareId);
PYBIND11_NUMPY_DTYPE(inl2_ned_sigma_t, timeOfWeekMs, StdPosNed, StdVelNed, StdAttNed, StdAccBias, StdGyrBias, StdBarBias, StdMagDeclination);
PYBIND11_NUMPY_DTYPE(strobe_in_time_t, week, timeOfWeekMs, pin, count);
PYBIND11_NUMPY_DTYPE(diag_msg_t, timeOfWeekMs, messageLength, message);
PYBIND11_NUMPY_DTYPE(survey_in_t, state, maxDurationSec, minAccuracy, elapsedTimeSec, hAccuracy, lla);
// PYBIND11_NUMPY_DTYPE(port_monitor_t, portNumber, txTimeMs, txBytesPerSec, rxTimeMs, rxBytesPerSec, status);
// PYBIND11_NUMPY_DTYPE(port_monitor_t, port);
// PYBIND11_NUMPY_DTYPE(evb2_t, week, timeOfWeekMs, firmwareVer, comBridgeCfg, loggerMode, loggerElapsedTimeMs, wifiSSID, wifiPSK, wifiIpAddr, serverIpAddr, serverPort);
// PYBIND11_NUMPY_DTYPE(evb_status_t, week, timeOfWeekMs, firmwareVer, evbStatus, loggerMode, loggerElapsedTimeMs, wifiIpAddr, sysCommand);
// PYBIND11_NUMPY_DTYPE(evb_flash_cfg_t, size, checksum, key, cbPreset, reserved1, cbf, cbOptions, bits, radioPID, radioNID, radioPowerLevel, wifi, server, encoderTickToWheelRad, CANbaud_kbps, can_receive_address, uinsComPort, uinsAuxPort, rtkd_unused24, portOptions, h3sp330BaudRate, h4xRadioBaudRate, h8gpioBaudRate);
PYBIND11_NUMPY_DTYPE(debug_array_t, i, f, lf);
PYBIND11_NUMPY_DTYPE(debug_string_t, s);
// PYBIND11_NUMPY_DTYPE(imu_mag_t, imu, mag);
// PYBIND11_NUMPY_DTYPE(pimu_mag_t, pimu, mag);
// PYBIND11_NUMPY_DTYPE(can_config_t, can_period_mult, can_transmit_address, can_baudrate_kbps, can_receive_address);

PYBIND11_NUMPY_DTYPE(gpx_gnss_status_t, lastRstCause, fwUpdateState, initState, runState);
PYBIND11_NUMPY_DTYPE(gpx_status_t, timeOfWeekMs, status, grmcBitsSer0, grmcBitsSer1, grmcBitsSer2, grmcBitsUSB, grmcNMEABitsSer0, grmcNMEABitsSer1, grmcNMEABitsSer2, grmcNMEABitsUSB, hdwStatus, mcuTemp, navOutputPeriodMs, flashCfgChecksum, rtkMode, gnssStatus, gpxSourcePort, upTime);

PYBIND11_NUMPY_DTYPE(gtime_t, time, sec);
PYBIND11_NUMPY_DTYPE(rtk_state_t, time, rp_ecef, rv_ecef, ra_ecef, bp_ecef, bv_ecef, qr, b, qb, sat_id);
PYBIND11_NUMPY_DTYPE(rtk_residual_t, time, nv, sat_id_i, sat_id_j, type, v);                    
PYBIND11_NUMPY_DTYPE(rtk_debug_t, time, rtkd_unused8_1, code_outlier, phase_outlier, rtkd_unused8_2, 
                        rtkd_unused8_3, rtkd_unused8_4, bad_baseline_holdamb, rtkd_unused8_5, 
                        outc_ovfl, rtkd_unused8_6, rtkd_unused8_7, large_v2b, 
                        base_position_update, rover_position_error, reset_bias, rtkd_unused8_8, 
                        rtkd_unused32_1, 
                        diff_age_error, rtkd_unused8_9, rover_packet_age_ms, base_packet_age_ms, 
                        rtkd_unused32_2, 
                        cycle_slips, 
                        rtk_to_rcvr_pos_error, 
                        rtkd_unused8_10, rtkd_unused8_11, error_count, error_code, 
                        rtkd_unused32_3, 
                        rtkd_unused8_12, rtkd_unused8_13, warning_count, warning_code, 
                        double_debug, 
                        debug, obs_base_unfiltered, obs_rover_unfiltered, 
                        rtkd_unused8_14, rtkd_unused8_15, rtkd_unused8_16, 
                        obs_unhealthy, obs_rover_relpos, obs_base_relpos, obs_pairs_used_float, obs_pairs_used_fixed, 
                        obs_eph_relpos, obs_low_snr_rover, obs_low_snr_base, rtkd_unused8_17, 
                        obs_zero_L1_rover, obs_zero_L1_base, obs_low_elev, rtkd_unused8_18, 
                        rtkd_unused8_19, rtkd_unused8_20, reserved
                    );
PYBIND11_NUMPY_DTYPE(obsd_t, time, sat, rcv, SNR, LLI, code, qualL, qualP, reserved, L, P, D);

PYBIND11_NUMPY_DTYPE(eph_t, sat, iode, iodc, sva, svh, week, code, flag, toe, toc, ttr, A, e, i0, OMG0, omg, M0, deln, OMGd, idot, crc, crs, cuc, cus, cic, cis, toes, fit, f0, f1, f2, tgd, Adot, ndot);
PYBIND11_NUMPY_DTYPE(geph_t, sat, iode, frq, svh, sva, age, toe, tof, pos, vel, acc, taun, gamn, dtaun);
PYBIND11_NUMPY_DTYPE(sbsmsg_t, week, tow, prn, msg, reserved);
PYBIND11_NUMPY_DTYPE(sta_t, deltype, pos, del, hgt, stationId);
PYBIND11_NUMPY_DTYPE(alm_t, sat, svh, svconf, week, toa, A, e, i0, OMG0, omg, M0, OMGd, toas, f0, f1);
PYBIND11_NUMPY_DTYPE(ion_model_utc_alm_t, ion_gps, ion_gal, ion_qzs, ion_cmp, ion_irn, utc_gps, utc_glo, utc_gal, utc_qzs, utc_cmp, utc_irn, utc_sbs, leaps, alm);


// Internal Data types
// PYBIND11_NUMPY_DTYPE(feature_bits_t, key, featureBits, hash1, hash2);
// PYBIND11_NUMPY_DTYPE(imu1_t, pqr, acc, mag);
// PYBIND11_NUMPY_DTYPE(sensor_bias_t, timeOfWeekMs, pqr, acc, mslBar, magI, magB);
// PYBIND11_NUMPY_DTYPE(hdw_param_imu_t, pqrDev, accDev, pqrSigma, accSigma, mean);
// PYBIND11_NUMPY_DTYPE(hdw_params_t, timeOfWeekMs, I, update, gpsCnoSigma, gpsCnoMean);
// PYBIND11_NUMPY_DTYPE(nvr_manage_t, flash_write_needed, flash_write_count);
// PYBIND11_NUMPY_DTYPE(inl2_misc_t, gps_time_last_valid);
// PYBIND11_NUMPY_DTYPE(rtk_debug_2_t, time, satBiasFloat, satBiasFix, qualL, sat, satBiasStd, satLockCnt, num_biases, reserved);
