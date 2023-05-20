#include <string>
#include "data_sets.h"
#include "ISGnss.h"

using namespace std;


string gnssIdToGnssName(int gnssId)
{
    switch (gnssId)
    {
    default:	                return to_string(gnssId);
    case SAT_SV_GNSS_ID_GPS:	return "GPS";
    case SAT_SV_GNSS_ID_SBS:	return "SBAS";
    case SAT_SV_GNSS_ID_GAL:	return "Galileo";
    case SAT_SV_GNSS_ID_BEI:	return "BeiDou";
    case SAT_SV_GNSS_ID_IME:	return "IMES";
    case SAT_SV_GNSS_ID_QZS:	return "QZSS";
    case SAT_SV_GNSS_ID_GLO:	return "GLONASS";
    case SAT_SV_GNSS_ID_IRN:	return "NavIC";
    }
}

char gnssIdToGnssPrefix(int gnssId)
{
    switch (gnssId)
    {
    default:	                return ' ';
    case SAT_SV_GNSS_ID_GPS:	return 'G';
    case SAT_SV_GNSS_ID_SBS:	return 'S';
    case SAT_SV_GNSS_ID_GAL:	return 'E';
    case SAT_SV_GNSS_ID_BEI:	return 'B';
    case SAT_SV_GNSS_ID_IME:	return 'I';
    case SAT_SV_GNSS_ID_QZS:	return 'Q';
    case SAT_SV_GNSS_ID_GLO:	return 'R';
    case SAT_SV_GNSS_ID_IRN:	return 'N';
    }
}

string gnssIdSigIdToSignalName(int gnssId, int sigId)
{
    switch(gnssId)
    {
    case SAT_SV_GNSS_ID_GPS:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_GPS_L1CA:        return "L1CA";
        case SAT_SV_SIG_ID_GPS_L2CL:        return "L2CL";
        case SAT_SV_SIG_ID_GPS_L2CM:        return "L2CM";
        case SAT_SV_SIG_ID_GPS_L5I:         return "L5I";
        case SAT_SV_SIG_ID_GPS_L5Q:         return "L5Q";
        }
        break;
    case SAT_SV_GNSS_ID_SBS:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_SBAS_L1CA:       return "L1CA";
        }
        break;
    case SAT_SV_GNSS_ID_GAL:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_Galileo_E1C2:    return "E1C2";
        case SAT_SV_SIG_ID_Galileo_E1B2:    return "E1B2";
        case SAT_SV_SIG_ID_Galileo_E5aI:    return "E5aI";
        case SAT_SV_SIG_ID_Galileo_E5aQ:    return "E5aQ";
        case SAT_SV_SIG_ID_Galileo_E5bI:    return "E5bI";
        case SAT_SV_SIG_ID_Galileo_E5bQ:    return "E5bQ";
        }
        break;
    case SAT_SV_GNSS_ID_BEI:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_BeiDou_B1D1:     return "B1D1";
        case SAT_SV_SIG_ID_BeiDou_B1D2:     return "B1D2";
        case SAT_SV_SIG_ID_BeiDou_B2D1:     return "B2D1";
        case SAT_SV_SIG_ID_BeiDou_B2D2:     return "B2D2";
        case SAT_SV_SIG_ID_BeiDou_B1C:      return "B1C";
        case SAT_SV_SIG_ID_BeiDou_B2a:      return "B2a";
        }
        break;
    case SAT_SV_GNSS_ID_QZS:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_QZSS_L1CA:       return "L1CA";
        case SAT_SV_SIG_ID_QZSS_L1S:        return "L1S";
        case SAT_SV_SIG_ID_QZSS_L2CM:       return "L2CM";
        case SAT_SV_SIG_ID_QZSS_L2CL: 		return "L2CL";
        case SAT_SV_SIG_ID_QZSS_L5I: 		return "L5I";
        case SAT_SV_SIG_ID_QZSS_L5Q: 		return "L5Q";
        }
        break;
    case SAT_SV_GNSS_ID_GLO:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_GLONASS_L1OF:	return "L1OF";
        case SAT_SV_SIG_ID_GLONASS_L2OF:	return "L2OF";
        }
        break;
    case SAT_SV_GNSS_ID_IRN:
        switch (sigId)
        {
        case SAT_SV_SIG_ID_NAVIC_L5A:		return "L5A";
        }
        break;
    }

    return "";
}
