#include "convert_ins.h"
#include "log_reader.h"

#define STRINGIZE(x) #x
#define STRINGIZE_VALUE_OF(x) STRINGIZE(x)
#define MESSAGE_VALUE(x) message(__FILE__ "(" STRINGIZE_VALUE_OF(__LINE__) "): " #x " = " STRINGIZE_VALUE_OF(x))
#define CONCAT_MESSAGE(text, value) message(__FILE__ "(" STRINGIZE_VALUE_OF(__LINE__) "): " text " = " STRINGIZE_VALUE_OF(value))

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
    std::ostringstream oss;

    oss << "LogReader Init:\n";

    // Print SDK protocol version
    oss << " - SDK Protocol: "
        << PROTOCOL_VERSION_CHAR0 << "."
        << PROTOCOL_VERSION_CHAR1 << "."
        << PROTOCOL_VERSION_CHAR2 << "."
        << PROTOCOL_VERSION_CHAR3 << "\n";

    std::vector<std::string> stl_serials = serials.cast<std::vector<std::string>>();
    oss << " - Loading from: " << log_directory << "\n";

    oss << " - Serials: ";
    for (const auto& s : stl_serials) {
        oss << s << "\n";
    }

    // Try loading log files
    bool loaded = false;
    if (logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_DAT, stl_serials)) {
        oss << " - Found *.dat log with ";
        loaded = true;
    } 
    else if (logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_RAW, stl_serials)) {
        oss << " - Found *.raw log with ";
        loaded = true;
    } 
    else if (logger_.LoadFromDirectory(log_directory, cISLogger::LOGTYPE_SDAT, stl_serials)) {
        oss << " - Found *.sdat log with ";
        loaded = true;
    } 
    else {
        oss << " - Unable to load files\n";
        std::cout << oss.str();
        return false;
    }

    oss << logger_.DeviceCount() << " device(s):";

    std::vector<int> serialNumbers;
    for (auto dev : logger_.DeviceLogs()) {
        oss << (serialNumbers.empty() ? " " : ", ") << dev->SerialNumber();
        serialNumbers.push_back(dev->SerialNumber());
    }
    oss << "\n";

    serialNumbers_ = py::cast(serialNumbers);
    g_python_parent = python_class;

    std::cout << oss.str();
    return true;
}

void LogReader::organizeData(shared_ptr<cDeviceLog> devLog)
{
    p_data_buf_t* data = NULL;
    while ((data = logger_.ReadData(devLog)))
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
        HANDLE_MSG( DID_GPS1_RCVR_POS, dev_log_->gps1UbxPos );
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
        HANDLE_MSG( DID_GPX_STATUS, dev_log_->gpxStatus );
        HANDLE_MSG( DID_GPX_DEBUG_ARRAY, dev_log_->gpxDebugArray );

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
    forward_message( DID_GPS1_RCVR_POS, dev_log_->gps1UbxPos, device_id );
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
    forward_message( DID_GPX_STATUS, dev_log_->gpxStatus, device_id );
    forward_message( DID_GPX_DEBUG_ARRAY, dev_log_->gpxDebugArray, device_id );
}

bool LogReader::load()
{
    // printf("LogReader::load() \n");

    std::vector<std::shared_ptr<cDeviceLog>> devices = logger_.DeviceLogs();
    for (int i = 0; i < (int)devices.size(); i++)
    {
        if (dev_log_ != nullptr)
        {
            delete dev_log_;
        }
        dev_log_ = new DeviceLog();

        organizeData(devices[i]);
        forwardData(i);
    }

	logger_.CloseAllFiles();

    return true;
}

pybind11::list LogReader::getSerialNumbers()
{ 
    return serialNumbers_; 
}

pybind11::list LogReader::protocolVersion()
{
#pragma CONCAT_MESSAGE("PROTOCOL_VERSION_CHAR0: ", PROTOCOL_VERSION_CHAR0)
#pragma CONCAT_MESSAGE("PROTOCOL_VERSION_CHAR1: ", PROTOCOL_VERSION_CHAR1)
#pragma CONCAT_MESSAGE("PROTOCOL_VERSION_CHAR2: ", PROTOCOL_VERSION_CHAR2)
#pragma CONCAT_MESSAGE("PROTOCOL_VERSION_CHAR3: ", PROTOCOL_VERSION_CHAR3)

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
    for (unsigned int i=0; i<dev_log_->ins1.size(); i++)
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
