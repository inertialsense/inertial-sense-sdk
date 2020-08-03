
#include "lunaFaultMaintenance.h"

//Check fault flags and act accordingly
void lunaFaultMaintenance()
{
    uint8_t faultStatus = (g_status.evbStatus & EVB_STATUS_LUNA_FAULT_MASK) >> EVB_STATUS_LUNA_FAULT_OFFSET;
    if (faultStatus & EVB_LUNA_FAULT_GEOFENCE_BOUNDARY_EXCEEDED)
    {
        ioport_set_pin_level(GPIO_10_PIN, IOPORT_PIN_LEVEL_HIGH);
    }
    
}

//Reset specific fault(s)
void lunaFaultReset(uint8_t fautlCode)
{
    g_status.evbStatus &= ~fautlCode; //TODO:Check this...
}

void checkGeofenceBoundary(double lla[3])
{
#ifdef ENABLE_EVB_LUNA
    if (lla[0] > g_flashCfg->maxLatGeofence || lla[0] < g_flashCfg->minLatGeofence || lla[1] > g_flashCfg->maxLonGeofence || lla[1] < g_flashCfg->minLonGeofence)
    {
        g_status.evbStatus = g_status.evbStatus | (EVB_LUNA_FAULT_GEOFENCE_BOUNDARY_EXCEEDED << EVB_STATUS_LUNA_FAULT_OFFSET);
        ioport_set_pin_level(GPIO_10_PIN, IOPORT_PIN_LEVEL_HIGH);
    }
	else
	{
		g_status.evbStatus = g_status.evbStatus & ~(EVB_LUNA_FAULT_GEOFENCE_BOUNDARY_EXCEEDED << EVB_STATUS_LUNA_FAULT_OFFSET);
		ioport_set_pin_level(GPIO_10_PIN, IOPORT_PIN_LEVEL_LOW);
	}
#endif
}	


