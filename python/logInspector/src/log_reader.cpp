#include "convert_ins.h"
#include "log_reader.h"

using namespace std;

static py::object g_python_parent;  // Including this inside LogReader class causes problems w/ garbage collection.

LogReader::LogReader()
{
    dev_log_ = nullptr;
}

LogReader::~LogReader()
{
    if (dev_log_ != nullptr)
    {
        delete dev_log_;
        dev_log_ = nullptr;
    }
}

template <>
void LogReader::log_message(int did, uint8_t* msg, std::vector<gps_raw_wrapper_t>& vec)
{
  gps_raw_t* raw_msg = (gps_raw_t*)msg;
  switch (raw_msg->dataType)
  {
  case raw_data_type_observation:
  {
    std::vector<obsd_t> obs{raw_msg->obsCount};
    for (int i = 0; i < raw_msg->obsCount; i++)
      obs.push_back(raw_msg->data.obs[i]);
    vec[0].obs.push_back(obs);
    break;
  }
  case raw_data_type_ephemeris:
    vec[0].eph.push_back(raw_msg->data.eph);
    break;
  case raw_data_type_glonass_ephemeris:
    vec[0].gloEph.push_back(raw_msg->data.gloEph);
    break;
  case raw_data_type_sbas:
    vec[0].sbas.push_back(raw_msg->data.sbas);
    break;
  case raw_data_type_base_station_antenna_position:
    vec[0].sta.push_back(raw_msg->data.sta);
    break;
  case raw_data_type_ionosphere_model_utc_alm:
    vec[0].ion.push_back(raw_msg->data.ion);
    break;
  default:
    break;
  }
}

template <typename T>
void LogReader::forward_message(eDataIDs did, std::vector<T>& vec, int device_id)
{
    g_python_parent.attr("did_callback")(did, py::array_t<T>(std::vector<ptrdiff_t>{(py::ssize_t)vec.size()}, vec.data()), device_id);
}

template <>
void LogReader::forward_message(eDataIDs did, std::vector<gps_raw_wrapper_t>& vec, int device_id)
{
    for (int i = 0; i < (int)vec[0].obs.size(); i++)
    {
        g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<obsd_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].obs[i].size()}, vec[0].obs[i].data()), device_id, (int)raw_data_type_observation);
    }
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<eph_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].eph.size()}, vec[0].eph.data()), device_id, (int)raw_data_type_ephemeris);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<geph_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].gloEph.size()}, vec[0].gloEph.data()), device_id, (int)raw_data_type_glonass_ephemeris);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<sbsmsg_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].sbas.size()}, vec[0].sbas.data()), device_id, (int)raw_data_type_sbas);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<ion_model_utc_alm_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].ion.size()}, vec[0].ion.data()), device_id, (int)raw_data_type_ionosphere_model_utc_alm);
    g_python_parent.attr("gps_raw_data_callback")(did, py::array_t<sta_t>(std::vector<ptrdiff_t>{(py::ssize_t)vec[0].sta.size()}, vec[0].sta.data()), device_id, (int)raw_data_type_base_station_antenna_position);
}


bool LogReader::init(py::object python_class, std::string log_directory, py::list serials)
{
    vector<string> stl_serials = serials.cast<vector<string>>();
    cout << "loading DAT file from " << log_directory << endl;
    cout << "reading serial numbers ";
    for (int i = 0; i < (int)stl_serials.size(); i++)
        cout << stl_serials[i] << "\n";

    // first try DAT files, if that doesn't work, then try SDAT files
    if (!logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_DAT, stl_serials))
    {
        cout << "unable to find DAT files, trying SDATS";
        if (!logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_SDAT, stl_serials))
            cout << "Unable to load files" << endl;
    }

    cout << "found " << logger_.GetDeviceCount() << " devices\n";
    vector<int> serialNumbers;
    for (int i = 0; i < (int)logger_.GetDeviceCount(); i++)
    {
        cout << (i==0 ? "  " : ", ") << logger_.GetDeviceInfo(i)->serialNumber;
        serialNumbers.push_back(logger_.GetDeviceInfo(i)->serialNumber);
    }
    cout << endl;
    serialNumbers_ = py::cast(serialNumbers);

    // python_parent_ = python_class;
    g_python_parent = python_class;
    return true;
}

void LogReader::organizeData(int device_id)
{
    p_data_t* data = NULL;
    while ((data = logger_.ReadData(device_id)))
    {
        // if (data->hdr.id == DID_DEV_INFO)
        //     volatile int debug = 0;

        if (data->hdr.size == 0)
            continue;

        switch (data->hdr.id)
        {

        // This is a helper macro, simply define the DID you want to forward,
        // as well as the datatype of that DID.  So long as the data type
        // has been defined in the PYBIND11_NUMPY_DTYPE macros below,
        // then this will work.  It uses templates to abstract a lot
        // of the tedium of this type of work
        #define HANDLE_MSG(DID, vec) \
        case DID: \
            log_message(data->hdr.id, data->buf, vec); \
            break;

        HANDLE_MSG( DID_DEV_INFO, dev_log_->devInfo );
        HANDLE_MSG( DID_SYS_FAULT, dev_log_->sysFault );
        HANDLE_MSG( DID_INS_1, dev_log_->ins1 );
        HANDLE_MSG( DID_INS_2, dev_log_->ins2 );
        HANDLE_MSG( DID_GPS1_UBX_POS, dev_log_->gps1UbxPos );
        HANDLE_MSG( DID_SYS_CMD, dev_log_->sysCmd );
        // HANDLE_MSG( DID_NMEA_BCAST_PERIOD, dev_log_->nmeaBcastPeriod );
        // HANDLE_MSG( DID_RMC, dev_log_->rmc );
        HANDLE_MSG( DID_SYS_PARAMS, dev_log_->sysParams );
        HANDLE_MSG( DID_SYS_SENSORS, dev_log_->sysSensors );
        HANDLE_MSG( DID_FLASH_CONFIG, dev_log_->flashCfg );
        HANDLE_MSG( DID_GPS1_POS, dev_log_->gps1Pos );
        HANDLE_MSG( DID_GPS2_POS, dev_log_->gps2Pos );
        HANDLE_MSG( DID_GPS1_SAT, dev_log_->gps1Sat );
        HANDLE_MSG( DID_GPS2_SAT, dev_log_->gps2Sat );
        HANDLE_MSG( DID_GPS1_VERSION, dev_log_->gps1Version );
        HANDLE_MSG( DID_GPS2_VERSION, dev_log_->gps2Version );
        HANDLE_MSG( DID_MAG_CAL, dev_log_->magCal );
        HANDLE_MSG( DID_INTERNAL_DIAGNOSTIC, dev_log_->internalDiagnostic );
        HANDLE_MSG( DID_GPS1_RTK_POS_REL, dev_log_->gps1RtkPosRel );
        HANDLE_MSG( DID_GPS1_RTK_POS_MISC, dev_log_->gps1RtkPosMisc );
        HANDLE_MSG( DID_GPS2_RTK_CMP_REL, dev_log_->gps1RtkCmpRel );
        HANDLE_MSG( DID_GPS2_RTK_CMP_MISC, dev_log_->gps1RtkCmpMisc );
        // HANDLE_MSG( DID_FEATURE_BITS, dev_log_->featureBits );
        HANDLE_MSG( DID_SENSORS_UCAL, dev_log_->sensorsUcal );
        HANDLE_MSG( DID_SENSORS_TCAL, dev_log_->sensorsTcal );
        HANDLE_MSG( DID_SENSORS_MCAL, dev_log_->sensorsMcal );
        HANDLE_MSG( DID_SENSORS_TC_BIAS, dev_log_->sensorsTcBias );
        HANDLE_MSG( DID_IO, dev_log_->io );
        // HANDLE_MSG( DID_SENSORS_ADC, dev_log_->sensorsAdc );
        HANDLE_MSG( DID_SCOMP, dev_log_->scomp );
        HANDLE_MSG( DID_REFERENCE_IMU, dev_log_->refImu );
        HANDLE_MSG( DID_REFERENCE_PIMU, dev_log_->refPImu );
        HANDLE_MSG( DID_REFERENCE_MAGNETOMETER, dev_log_->refMag );
        HANDLE_MSG( DID_GPS1_VEL, dev_log_->gps1Vel );
        HANDLE_MSG( DID_GPS2_VEL, dev_log_->gps2Vel );
        // HANDLE_MSG( DID_HDW_PARAMS, dev_log_->hdwParams );
        // HANDLE_MSG( DID_NVR_MANAGE_USERPAGE, dev_log_->nvrManageUserpage );
        // HANDLE_MSG( DID_NVR_USERPAGE_SN, dev_log_->nvrUserpageSn );
        // HANDLE_MSG( DID_NVR_USERPAGE_G0, dev_log_->nvrUserpageG0 );
        // HANDLE_MSG( DID_NVR_USERPAGE_G1, dev_log_->nvrUserpageG1 );
        // HANDLE_MSG( DID_RTOS_INFO, dev_log_->rtosInfo );
        HANDLE_MSG( DID_DEBUG_STRING, dev_log_->debugString );
        HANDLE_MSG( DID_DEBUG_ARRAY, dev_log_->debugArray );
        // HANDLE_MSG( DID_CAL_SC, dev_log_->calSc );
        // HANDLE_MSG( DID_CAL_SC1, dev_log_->calSc1 );
        // HANDLE_MSG( DID_CAL_SC2, dev_log_->calSc2 );
        HANDLE_MSG( DID_SENSORS_ADC_SIGMA, dev_log_->sensorsAdcSigma );
        HANDLE_MSG( DID_INL2_STATES, dev_log_->inl2States );
        HANDLE_MSG( DID_INL2_STATUS, dev_log_->inl2Status );
        // HANDLE_MSG( DID_INL2_MISC, dev_log_->inl2Misc );
        HANDLE_MSG( DID_MAGNETOMETER, dev_log_->magnetometer );
        HANDLE_MSG( DID_BAROMETER, dev_log_->barometer );
        HANDLE_MSG( DID_GPS1_RTK_POS, dev_log_->gps1RtkPos );
        HANDLE_MSG( DID_IMU3_UNCAL, dev_log_->imu3Uncal );
        HANDLE_MSG( DID_IMU3_RAW, dev_log_->imu3Raw );
        HANDLE_MSG( DID_IMU_RAW, dev_log_->imuRaw );
        HANDLE_MSG( DID_PIMU, dev_log_->pimu );
        HANDLE_MSG( DID_IMU, dev_log_->imu );
        HANDLE_MSG( DID_INL2_MAG_OBS_INFO, dev_log_->inl2MagObsInfo );
        HANDLE_MSG( DID_GPS_BASE_RAW, dev_log_->gpsBaseRaw );
        // HANDLE_MSG( DID_GPS_RTK_OPT, dev_log_->gpsRtkOpt );
        HANDLE_MSG( DID_MANUFACTURING_INFO, dev_log_->manufacturingInfo );
        HANDLE_MSG( DID_BIT, dev_log_->bit );
        HANDLE_MSG( DID_INS_3, dev_log_->ins3 );
        HANDLE_MSG( DID_INS_4, dev_log_->ins4 );
        HANDLE_MSG( DID_INL2_NED_SIGMA, dev_log_->inl2NedSigma );
        HANDLE_MSG( DID_STROBE_IN_TIME, dev_log_->strobeInTime );
        HANDLE_MSG( DID_GPS1_RAW, dev_log_->gps1Raw );
        HANDLE_MSG( DID_GPS2_RAW, dev_log_->gps2Raw );
        HANDLE_MSG( DID_WHEEL_ENCODER, dev_log_->wheelEncoder );
        HANDLE_MSG( DID_GROUND_VEHICLE, dev_log_->groundVehicle );
        HANDLE_MSG( DID_EVB_LUNA_VELOCITY_CONTROL, dev_log_->evbVelocityControl );
        HANDLE_MSG( DID_DIAGNOSTIC_MESSAGE, dev_log_->diagnosticMessage );
        HANDLE_MSG( DID_SURVEY_IN, dev_log_->surveyIn );
        // HANDLE_MSG( DID_EVB2, dev_log_->evb2 );
        // HANDLE_MSG( DID_PORT_MONITOR, dev_log_->portMonitor );
        // HANDLE_MSG( DID_RTK_STATE, dev_log_->rtkState);
        HANDLE_MSG( DID_RTK_CODE_RESIDUAL, dev_log_->rtkCodeResidual);
        HANDLE_MSG( DID_RTK_PHASE_RESIDUAL, dev_log_->rtkPhaseResidual);
        HANDLE_MSG( DID_RTK_DEBUG, dev_log_->rtkDebug);
        // HANDLE_MSG( DID_RTK_DEBUG_2, dev_log_->rtkDebug2);

        default:
            //            printf("Unhandled IS message DID: %d\n", message_type);
            break;
        }
    }
}

void LogReader::forwardData(int device_id)
{
    forward_message( DID_DEV_INFO, dev_log_->devInfo , device_id);
    forward_message( DID_SYS_FAULT, dev_log_->sysFault, device_id );
    forward_message( DID_INS_1, dev_log_->ins1, device_id );
    forward_message( DID_INS_2, dev_log_->ins2, device_id );
    forward_message( DID_GPS1_UBX_POS, dev_log_->gps1UbxPos, device_id );
    forward_message( DID_SYS_CMD, dev_log_->sysCmd, device_id );
    // forward_message( DID_NMEA_BCAST_PERIOD, dev_log_->nmeaBcastPeriod, device_id );
    // forward_message( DID_RMC, dev_log_->rmc, device_id );
    forward_message( DID_SYS_PARAMS, dev_log_->sysParams, device_id );
    forward_message( DID_SYS_SENSORS, dev_log_->sysSensors, device_id );
    forward_message( DID_FLASH_CONFIG, dev_log_->flashCfg, device_id );
    forward_message( DID_GPS1_POS, dev_log_->gps1Pos, device_id );
    forward_message( DID_GPS2_POS, dev_log_->gps2Pos, device_id );
    forward_message( DID_GPS1_SAT, dev_log_->gps1Sat, device_id );
    forward_message( DID_GPS2_SAT, dev_log_->gps2Sat, device_id );
    forward_message( DID_GPS1_VERSION, dev_log_->gps1Version, device_id );
    forward_message( DID_GPS2_VERSION, dev_log_->gps2Version, device_id );
    forward_message( DID_MAG_CAL, dev_log_->magCal, device_id );
    forward_message( DID_INTERNAL_DIAGNOSTIC, dev_log_->internalDiagnostic, device_id );
    forward_message( DID_GPS1_RTK_POS_REL, dev_log_->gps1RtkPosRel, device_id );
    forward_message( DID_GPS1_RTK_POS_MISC, dev_log_->gps1RtkPosMisc, device_id );
    forward_message( DID_GPS2_RTK_CMP_REL, dev_log_->gps1RtkCmpRel, device_id );
    forward_message( DID_GPS2_RTK_CMP_MISC, dev_log_->gps1RtkCmpMisc, device_id );
    // forward_message( DID_FEATURE_BITS, dev_log_->featureBits, device_id );
    forward_message( DID_SENSORS_UCAL, dev_log_->sensorsUcal, device_id );
    forward_message( DID_SENSORS_TCAL, dev_log_->sensorsTcal, device_id );
    forward_message( DID_SENSORS_MCAL, dev_log_->sensorsMcal, device_id );
    forward_message( DID_SENSORS_TC_BIAS, dev_log_->sensorsTcBias, device_id );
    forward_message( DID_IO, dev_log_->io, device_id );
    // forward_message( DID_SENSORS_ADC, dev_log_->sensorsAdc, device_id );
    forward_message( DID_SCOMP, dev_log_->scomp, device_id );
    forward_message( DID_REFERENCE_IMU, dev_log_->refImu, device_id );
    forward_message( DID_REFERENCE_PIMU, dev_log_->refPImu, device_id );
    forward_message( DID_REFERENCE_MAGNETOMETER, dev_log_->refMag, device_id );
    forward_message( DID_GPS1_VEL, dev_log_->gps1Vel, device_id );
    forward_message( DID_GPS2_VEL, dev_log_->gps2Vel, device_id );
    // forward_message( DID_HDW_PARAMS, dev_log_->hdwParams, device_id );
    // forward_message( DID_NVR_MANAGE_USERPAGE, dev_log_->nvrManageUserpage, device_id );
    // forward_message( DID_NVR_USERPAGE_SN, dev_log_->nvrUserpageSn, device_id );
    // forward_message( DID_NVR_USERPAGE_G0, dev_log_->nvrUserpageG0, device_id );
    // forward_message( DID_NVR_USERPAGE_G1, dev_log_->nvrUserpageG1, device_id );
    // forward_message( DID_RTOS_INFO, dev_log_->rtosInfo, device_id );
    forward_message( DID_DEBUG_STRING, dev_log_->debugString, device_id );
    forward_message( DID_DEBUG_ARRAY, dev_log_->debugArray, device_id );
    // forward_message( DID_CAL_SC, dev_log_->calSc, device_id );
    // forward_message( DID_CAL_SC1, dev_log_->calSc1, device_id );
    // forward_message( DID_CAL_SC2, dev_log_->calSc2, device_id );
    forward_message( DID_SENSORS_ADC_SIGMA, dev_log_->sensorsAdcSigma, device_id );
    forward_message( DID_INL2_STATES, dev_log_->inl2States, device_id );
    forward_message( DID_INL2_STATUS, dev_log_->inl2Status, device_id );
    // forward_message( DID_INL2_MISC, dev_log_->inl2Misc, device_id );
    forward_message( DID_MAGNETOMETER, dev_log_->magnetometer, device_id );
    forward_message( DID_BAROMETER, dev_log_->barometer, device_id );
    forward_message( DID_GPS1_RTK_POS, dev_log_->gps1RtkPos, device_id );
    forward_message( DID_IMU3_UNCAL, dev_log_->imu3Uncal, device_id );
    forward_message( DID_IMU3_RAW, dev_log_->imu3Raw, device_id );
    forward_message( DID_IMU_RAW, dev_log_->imuRaw, device_id );
    forward_message( DID_PIMU, dev_log_->pimu, device_id );
    forward_message( DID_IMU, dev_log_->imu, device_id );
    forward_message( DID_INL2_MAG_OBS_INFO, dev_log_->inl2MagObsInfo, device_id );
    forward_message( DID_GPS_BASE_RAW, dev_log_->gpsBaseRaw, device_id );
    // forward_message( DID_GPS_RTK_OPT, dev_log_->gpsRtkOpt, device_id );
    forward_message( DID_MANUFACTURING_INFO, dev_log_->manufacturingInfo, device_id );
    forward_message( DID_BIT, dev_log_->bit, device_id );
    forward_message( DID_INS_3, dev_log_->ins3, device_id );
    forward_message( DID_INS_4, dev_log_->ins4, device_id );
    forward_message( DID_INL2_NED_SIGMA, dev_log_->inl2NedSigma, device_id );
    forward_message( DID_STROBE_IN_TIME, dev_log_->strobeInTime, device_id );
    forward_message( DID_GPS1_RAW, dev_log_->gps1Raw, device_id );
    forward_message( DID_GPS2_RAW, dev_log_->gps2Raw, device_id );
    forward_message( DID_WHEEL_ENCODER, dev_log_->wheelEncoder, device_id );
    forward_message( DID_GROUND_VEHICLE, dev_log_->groundVehicle, device_id );
    forward_message( DID_EVB_LUNA_VELOCITY_CONTROL, dev_log_->evbVelocityControl, device_id );
    forward_message( DID_DIAGNOSTIC_MESSAGE, dev_log_->diagnosticMessage, device_id );
    forward_message( DID_SURVEY_IN, dev_log_->surveyIn, device_id );
    // forward_message( DID_EVB2, dev_log_->evb2, device_id );
    // forward_message( DID_PORT_MONITOR, dev_log_->portMonitor, device_id );

    // forward_message( DID_RTK_STATE, dev_log_->rtkState, device_id);
    forward_message( DID_RTK_CODE_RESIDUAL, dev_log_->rtkCodeResidual, device_id);
    forward_message( DID_RTK_PHASE_RESIDUAL, dev_log_->rtkPhaseResidual, device_id);
    forward_message( DID_RTK_DEBUG, dev_log_->rtkDebug, device_id);
    // forward_message( DID_RTK_DEBUG_2, dev_log_->rtkDebug2, device_id);
}

bool LogReader::load()
{
    for (int i = 0; i < (int)logger_.GetDeviceCount(); i++)
    {
        if (dev_log_ != nullptr)
        {
            delete dev_log_;
        }
        dev_log_ = new DeviceLog();

        organizeData(i);
        forwardData(i);
    }

    return true;
}

pybind11::list LogReader::getSerialNumbers()
{ 
    return serialNumbers_; 
}

pybind11::list LogReader::protocolVersion()
{ 
    vector<int> version;
    version.push_back(PROTOCOL_VERSION_CHAR0);
    version.push_back(PROTOCOL_VERSION_CHAR1);
    version.push_back(PROTOCOL_VERSION_CHAR2);
    version.push_back(PROTOCOL_VERSION_CHAR3);
    return py::cast(version);
}

void LogReader::ins1ToIns2(int device_id)
{
    printf("LogReader::ins1ToIns2() converting ins1 to ins2 for device: %d\n", device_id);
    ins_2_t ins2;
    dev_log_->ins2.clear();
    for (int i=0; i<dev_log_->ins1.size(); i++)
    {
        convertIns1ToIns2(&(dev_log_->ins1[i]), &ins2);
        dev_log_->ins2.push_back(ins2);
    }
    forward_message( DID_INS_2, dev_log_->ins2, device_id );
}

void LogReader::exitHack(int exit_code)
{
    // Nasty hack
    exit(exit_code);
}

// Look at the pybind documentation to understand what is going on here.
// Don't change anything unless you know what you are doing
PYBIND11_MODULE(log_reader, m) {
    // Declare a module (the .so file compiled by CMake needs to have the same
    // name as defined here.  Don't change the name without changing the related
    // lines in the CmakeLists.txt file.  We can import this module in python
    // with "import log_reader" so long as the log_reader.so file is in the python path
    m.doc() = "log_reader";

    // Bind the Interface Class
    py::class_<LogReader>(m, "LogReader") // The object will be named IS_Comm in python
            .def(py::init<>()) // constructor
            .def("init", &LogReader::init)
            .def("load", &LogReader::load)
            .def("getSerialNumbers", &LogReader::getSerialNumbers)
            .def("protocolVersion", &LogReader::protocolVersion)
            .def("ins1ToIns2", &LogReader::ins1ToIns2)
            .def("exitHack", &LogReader::exitHack);

#include "pybindMacros.h"
}
