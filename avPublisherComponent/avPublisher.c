#include "legato.h"
#include "interfaces.h"
#include "gps.h"

#define LOCATION_MAX_CELLS		15
#define LOCATION_MAX_AP			15

//--------------------------------------------------------------------------------------------------
/*
 * command definitions
 */
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
/*
 * type definitions
 */
//--------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------
/**
 * An abstract representation of a sensor
 *
 * Values are represented as void* because different sensors may produce double, uint32_t or a
 * custom struct type. The only requirement is that all of the functions must expect the same type
 * of value.
 */
//--------------------------------------------------------------------------------------------------
struct Item
{
    // A human readable name for the sensor
    const char *name;

    // Reads a value from the sensor
    le_result_t (*read)(void *value);

    // Checks to see if the value read exceeds the threshold relative to the last recorded value. If
    // the function returns true, the readValue will be recorded.
    bool (*thresholdCheck)(const void *recordedValue, const void *readValue);

    // Records the value into the given record.
    le_result_t (*record)(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);

    // Copies the sensor reading from src to dest. The management of the src and dest data is the
    // responsibility of the caller.
    void (*copyValue)(void *dest, const void *src);

    // Most recently read value from the sensor. Should be initialized to point at a variable large
    // enough to store a sensor reading.
    void *lastValueRead;

    // Most recently recorded value from the sensor. Must be initialized to point to a variable to
    // store a reading and must be a differnt variable from what lastValueRead is pointing at.
    void *lastValueRecorded;

    // Time when the last reading was recorded.
    uint64_t lastTimeRecorded;

    // Time when the last reading was read.
    uint64_t lastTimeRead;
};

struct Location3d
{
    double latitude;
    double longitude;
    double hAccuracy;
    double altitude;
    double vAccuracy;
};

//--------------------------------------------------------------------------------------------------
/**
 * Current location.
 */
//--------------------------------------------------------------------------------------------------
struct Location
{
    struct {
        char mcc[LE_MRC_MCC_BYTES];
        char mnc[LE_MRC_MNC_BYTES];
        le_mrc_Rat_t rat;
	uint32_t cid;
    	uint16_t lac;
	int32_t signal;
    } servingCellInfo;
    struct {
        char mcc[LE_MRC_MCC_BYTES];
        char mnc[LE_MRC_MNC_BYTES];
        uint32_t cid;
    } pciCellInfo[LOCATION_MAX_CELLS];
    uint32_t numPciCells;
};

//--------------------------------------------------------------------------------------------------
/**
 * A data structure that stores a single reading from all of the sensors.
 */
//--------------------------------------------------------------------------------------------------
struct LocationReadings
{
    struct Location location;
#ifdef GPS_ENABLE
    struct Location3d location;
#endif // GPS_ENABLE
};

//--------------------------------------------------------------------------------------------------
/*
 * static function declarations
 */
//--------------------------------------------------------------------------------------------------
static void PushCallbackHandler(le_avdata_PushStatus_t status, void* context);
static uint64_t GetCurrentTimestamp(void);
static void SampleTimerHandler(le_timer_Ref_t timer);

static le_result_t LocationRead(void *value);
static bool LocationThreshold(const void *recordedValue, const void* readValue);
static le_result_t LocationRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void LocationCopyValue(void *dest, const void *src);

#ifdef GPS_ENABLE
static le_result_t GpsRead(void *value);
static bool GpsThreshold(const void *recordedValue, const void* readValue);
static le_result_t GpsRecord(le_avdata_RecordRef_t ref, uint64_t timestamp, void *value);
static void GpsCopyValue(void *dest, const void *src);
#endif // GPS_ENABLE

static void AvSessionStateHandler (le_avdata_SessionState_t state, void *context);

//--------------------------------------------------------------------------------------------------
/*
 * variable definitions
 */
//--------------------------------------------------------------------------------------------------

// Wait time between each round of sensor readings.
static const int DelayBetweenReadings = 30;

// The maximum amount of time to wait for a reading to exceed a threshold before a publish is
// forced.
static const int MaxIntervalBetweenPublish = 120;

// The minimum amount of time to wait between publishing data.
static const int MinIntervalBetweenPublish = 10;

// How old the last published value must be for an item to be considered stale. The next time a
// publish occurs, the most recent reading of all stale items will be published.
static const int TimeToStale = 60;

static le_timer_Ref_t SampleTimer;
static le_avdata_RequestSessionObjRef_t AvSession;
static le_avdata_RecordRef_t RecordRef;
static le_avdata_SessionStateHandlerRef_t HandlerRef;

static bool DeferredPublish = false;
static uint64_t LastTimePublished = 0;


//--------------------------------------------------------------------------------------------------
/*
 * Data storage for sensor readings.
 *
 * This struct contains the most recently read values from the sensors and the most recently
 * recorded values from the sensors.
 */
//--------------------------------------------------------------------------------------------------
static struct
{
    struct LocationReadings recorded;  // sensor values most recently recorded
    struct LocationReadings read;      // sensor values most recently read
} LocationData;


//--------------------------------------------------------------------------------------------------
/**
 * An array representing all of the sensor values to read and publish
 */
//--------------------------------------------------------------------------------------------------
struct Item Items[] =
{
    {
        .name = "location",
        .read = LocationRead,
        .thresholdCheck = LocationThreshold,
        .record = LocationRecord,
        .copyValue = LocationCopyValue,
	.lastValueRead = &LocationData.read.location,
        .lastValueRecorded = &LocationData.recorded.location,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
#ifdef GPS_ENABLE
    {
        .name = "gps",
        .read = GpsRead,
        .thresholdCheck = GpsThreshold,
        .record = GpsRecord,
        .copyValue = GpsCopyValue,
        .lastValueRead = &LocationData.read.location,
        .lastValueRecorded = &LocationData.recorded.location,
        .lastTimeRead = 0,
        .lastTimeRecorded = 0,
    },
#endif // GPS_ENABLE
};


//--------------------------------------------------------------------------------------------------
/*
 * static function definitions
 */
//--------------------------------------------------------------------------------------------------


//--------------------------------------------------------------------------------------------------
/**
 * Handles notification of LWM2M push status.
 *
 * This function will warn if there is an error in pushing data, but it does not make any attempt to
 * retry pushing the data.
 */
//--------------------------------------------------------------------------------------------------
static void PushCallbackHandler
(
    le_avdata_PushStatus_t status, ///< Push success/failure status
    void* context                  ///< Not used
)
{
    switch (status)
    {
    case LE_AVDATA_PUSH_SUCCESS:
        // data pushed successfully
        break;

    case LE_AVDATA_PUSH_FAILED:
        LE_WARN("Push was not successful");
        break;

    default:
        LE_ERROR("Unhandled push status %d", status);
        break;
    }
}


//--------------------------------------------------------------------------------------------------
/**
 * Convenience function to get current time as uint64_t.
 *
 * @return
 *      Current time as a uint64_t
 */
//--------------------------------------------------------------------------------------------------
static uint64_t GetCurrentTimestamp(void)
{
    struct timeval tv;
    gettimeofday(&tv, NULL);
    uint64_t utcMilliSec = (uint64_t)(tv.tv_sec) * 1000 + (uint64_t)(tv.tv_usec) / 1000;
    return utcMilliSec;
}


//--------------------------------------------------------------------------------------------------
/**
 * Handler for the sensor sampling timer
 *
 * Each time this function is called due to timer expiry each sensor described in the Items array
 * will be read. If any sensor item's thresholdCheck() function returns true, then that reading is
 * recorded and a publish action is scheduled. The data will be published immediately unless fewer
 * than MinIntervalBetweenPublish seconds have elapsed since the last publish. If that is the case,
 * the publish will be deferred until the minimum wait has elapsed. If no publish has occurred for
 * MaxIntervalBetweenPublish seconds, then a publish is forced. When a push is about to be executed
 * the list of items is checked again for any entries which have not been recorded in greater than
 * TimeToStale seconds. Stale items are recorded and then the record is published.
 */
//--------------------------------------------------------------------------------------------------
static void SampleTimerHandler
(
    le_timer_Ref_t timer  ///< Sensor sampling timer
)
{
    uint64_t now = GetCurrentTimestamp();
    bool publish = false;

    for (int i = 0; i < NUM_ARRAY_MEMBERS(Items); i++)
    {
        le_result_t r;
        struct Item *it = &Items[i];
        r = it->read(it->lastValueRead);
        if (r == LE_OK)
        {
            it->lastTimeRead = now;
            if (it->lastTimeRecorded == 0 || it->thresholdCheck(it->lastValueRead, it->lastValueRecorded))
            {
                r = it->record(RecordRef, now, it->lastValueRead);
                if (r == LE_OK)
                {
                    it->copyValue(it->lastValueRecorded, it->lastValueRead);
                    publish = true;
                }
                else
                {
                    LE_WARN("Failed to record %s", it->name);
                }
            }
        }
        else
        {
            LE_WARN("Failed to read %s", it->name);
        }

        if ((now - it->lastTimeRecorded) > (MaxIntervalBetweenPublish * 1000) &&
            it->lastTimeRead > LastTimePublished)
        {
            publish = true;
        }
    }

    if (publish || DeferredPublish)
    {
        if ((now - LastTimePublished) < MinIntervalBetweenPublish)
        {
            DeferredPublish = true;
        }
        else
        {
            // Find all of the stale items and record their current reading
            for (int i = 0; i < NUM_ARRAY_MEMBERS(Items); i++)
            {
                struct Item *it = &Items[i];
                if ((now - it->lastTimeRecorded) > (TimeToStale * 1000) &&
                    it->lastTimeRead > it->lastTimeRecorded)
                {
                    le_result_t r = it->record(RecordRef, it->lastTimeRead, it->lastValueRead);
                    if (r == LE_OK)
                    {
                        it->copyValue(it->lastValueRecorded, it->lastValueRead);
                        it->lastTimeRecorded = it->lastTimeRead;
                    }
                    else
                    {
                        LE_WARN("Failed to record %s", it->name);
                    }
                }
            }

            le_result_t r = le_avdata_PushRecord(RecordRef, PushCallbackHandler, NULL);
            if (r != LE_OK)
            {
                LE_ERROR("Failed to push record - %s", LE_RESULT_TXT(r));
            }
            else
            {
                LastTimePublished = now;
                DeferredPublish = false;
            }
        }
    }
}

//--------------------------------------------------------------------------------------------------
/**
 * Read the location PCI information
 *
 * @return
 *      N/A.
 */
//--------------------------------------------------------------------------------------------------
static void LocationPciScan
(
    void *value  ///< Pointer to a struct Location to store the reading in
)
{
    struct Location *v = value;
    le_result_t res;

    v->numPciCells = 0;
    le_mrc_ScanInformationListRef_t scanInformationListRef = le_mrc_PerformCellularNetworkPciScan(LE_MRC_BITMASK_RAT_LTE);
    if (scanInformationListRef)
    {
        le_mrc_ScanInformationRef_t scanInformationRef = le_mrc_GetFirstCellularNetworkScan(scanInformationListRef);
        while (scanInformationRef && (v->numPciCells < LOCATION_MAX_CELLS))
        {
            res = le_mrc_GetCellularNetworkMccMnc(scanInformationRef, 
                v->pciCellInfo[v->numPciCells].mcc, sizeof(v->pciCellInfo[v->numPciCells].mcc), 
                v->pciCellInfo[v->numPciCells].mnc, sizeof(v->pciCellInfo[v->numPciCells].mnc));
            if (res != LE_OK) 
            {
                LE_ERROR("ERROR: le_mrc_GetCellularNetworkMccMnc() failed(%d)", res);
                goto cleanup;
            }

            res = le_mrc_GetCellularNetworkGlobalCellId(scanInformationRef, &v->pciCellInfo[v->numPciCells].cid);
            if (res != LE_OK) 
            {
                LE_ERROR("ERROR: le_mrc_GetCellularNetworkGlobalCellId() failed(%d)", res);
                goto cleanup;
            }

            LE_INFO("CID(0x%08x) MCC('%s') MNC('%s')", 
                v->pciCellInfo[v->numPciCells].cid, v->pciCellInfo[v->numPciCells].mcc, v->pciCellInfo[v->numPciCells].mnc);
            v->numPciCells++;
            scanInformationRef = le_mrc_GetNextCellularNetworkScan(scanInformationListRef);
        }
    }

cleanup:
    if (scanInformationListRef)
    {
        le_mrc_DeleteCellularNetworkScan(scanInformationListRef);
    }

    return;
}

//--------------------------------------------------------------------------------------------------
/**
 * Read the location
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t LocationRead
(
    void *value  ///< Pointer to a struct Location to store the reading in
)
{
    struct Location *v = value;
    le_mrc_MetricsRef_t metricsRef;
    le_result_t res = LE_OK;

    res = le_mrc_GetCurrentNetworkMccMnc(v->servingCellInfo.mcc, sizeof(v->servingCellInfo.mcc), 
        v->servingCellInfo.mnc, sizeof(v->servingCellInfo.mnc));	
    if (res != LE_OK) 
    {
        LE_ERROR("ERROR: le_mrc_GetCurrentNetworkMccMnc() failed(%d)", res);
    }

    res = le_mrc_GetRadioAccessTechInUse(&v->servingCellInfo.rat);
    if (res != LE_OK) 
    {
        LE_ERROR("ERROR: le_mrc_GetRadioAccessTechInUse() failed(%d)", res);
    }

    v->servingCellInfo.cid = le_mrc_GetServingCellId();
    v->servingCellInfo.lac = le_mrc_GetServingCellLocAreaCode();

    metricsRef = le_mrc_MeasureSignalMetrics();
    if (metricsRef)
    {
        switch (v->servingCellInfo.rat) 
        {
        case LE_MRC_RAT_GSM:
        {
            int32_t rssi;
            uint32_t ber;
	    res = le_mrc_GetGsmSignalMetrics(metricsRef, &rssi, &ber);
            if (res != LE_OK) 
            {
                LE_ERROR("ERROR: le_mrc_GetGsmSignalMetrics() failed(%d)", res);
            }
            else
                v->servingCellInfo.signal = rssi;
            break;
        }
        case LE_MRC_RAT_UMTS:
        {
            int32_t ss;
            uint32_t bler;
            int32_t ecio;
            int32_t rscp;
            int32_t sinr;
            res = le_mrc_GetUmtsSignalMetrics(metricsRef, &ss, &bler, &ecio, &rscp, &sinr);
            if (res != LE_OK) 
            {
                LE_ERROR("ERROR: le_mrc_GetUmtsSignalMetrics() failed(%d)", res);
            }
            else
                v->servingCellInfo.signal = ss;
            break;
        }
        case LE_MRC_RAT_LTE:
        {
            int32_t ss;
            uint32_t bler; 
            int32_t rsrq; 
            int32_t rsrp; 
            int32_t sinr;
            res = le_mrc_GetLteSignalMetrics(metricsRef, &ss, &bler, &rsrq, &rsrp, &sinr);
            if (res != LE_OK) 
            {
                LE_ERROR("ERROR: le_mrc_GetLteSignalMetrics() failed(%d)", res);
            }
            else
                v->servingCellInfo.signal = ss;
            break;
        }
        case LE_MRC_RAT_CDMA:
        {
            int32_t ss; 
            uint32_t er; 
            int32_t ecio; 
            int32_t sinr; 
            int32_t io;
            res = le_mrc_GetCdmaSignalMetrics(metricsRef, &ss, &er, &ecio, &sinr, &io);
            if (res != LE_OK) 
            {
                LE_ERROR("ERROR: le_mrc_GetLteSignalMetrics() failed(%d)", res);
            }
            else
                v->servingCellInfo.signal = ss;
            break;
        }
        default:
            LE_WARN("WARNING: unhandled RAT(%d)", v->servingCellInfo.rat);
            break;
        }

        le_mrc_DeleteSignalMetrics(metricsRef);
    }

    LE_INFO("Serving RAT(%d) MCC/MNC('%s'/'%s') cellId(0x%08x) lac(0x%04x) signal(%d)", 
        v->servingCellInfo.rat, v->servingCellInfo.mcc, v->servingCellInfo.mnc, 
        v->servingCellInfo.cid, v->servingCellInfo.lac, v->servingCellInfo.signal);

    LocationPciScan(v);
    return LE_OK;
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the location has changed sufficiently to warrant recording of a new
 * reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool LocationThreshold
(
    const void *recordedValue, ///< Last recorded location
    const void *readValue      ///< Most recent location
)
{
    const struct Location *v1 = recordedValue;
    const struct Location *v2 = readValue;
    
    if (v1->numPciCells != v2->numPciCells) return true;
    else 
    {
        uint32_t i = 0;
        while (i < v1->numPciCells) 
	{
	    if (strncmp(v1->pciCellInfo[i].mcc, v2->pciCellInfo[i].mcc, sizeof(v1->pciCellInfo[i].mcc)) ||
                strncmp(v1->pciCellInfo[i].mnc, v2->pciCellInfo[i].mnc, sizeof(v1->pciCellInfo[i].mnc)) ||
                (v1->pciCellInfo[i].cid != v2->pciCellInfo[i].cid))
		return true;
	    i++;
	}
    }

    return false;
}

//--------------------------------------------------------------------------------------------------
/**
 * Records location at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t LocationRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The struct Gyro value to record
)
{
    char node[128];
    struct Location *v = value;
    le_result_t result = LE_FAULT;
    uint32_t i = 0;
    
    if (strlen(v->servingCellInfo.mcc) > 0) 
    {
        result = le_avdata_RecordString(RecordRef, "Location.ServingCellInfo.MCC", v->servingCellInfo.mcc, timestamp);
        if (result != LE_OK)
        {
            LE_ERROR("Couldn't record location cell ID reading - %s", LE_RESULT_TXT(result));
            goto done;
        }
    }

    if (strlen(v->servingCellInfo.mnc) > 0) 
    {
        result = le_avdata_RecordString(RecordRef, "Location.ServingCellInfo.MNC", v->servingCellInfo.mnc, timestamp);
        if (result != LE_OK)
        {
            LE_ERROR("Couldn't record location cell ID reading - %s", LE_RESULT_TXT(result));
            goto done;
        }
    }

    if (v->servingCellInfo.cid != -1)
    {
	result = le_avdata_RecordInt(RecordRef, "Location.ServingCellInfo.Cid", v->servingCellInfo.cid, timestamp);
        if (result != LE_OK)
        {
            LE_ERROR("Couldn't record location cell ID reading - %s", LE_RESULT_TXT(result));
            goto done;
        }
    }

    if ((v->servingCellInfo.lac != UINT16_MAX) && (v->servingCellInfo.lac != -1))
    {
	result = le_avdata_RecordInt(RecordRef, "Location.ServingCellInfo.Lac", v->servingCellInfo.lac, timestamp);
        if (result != LE_OK)
        {
            LE_ERROR("Couldn't record location LAC reading - %s", LE_RESULT_TXT(result));
            goto done;
        }
    }

    if (v->servingCellInfo.signal)
    {
	result = le_avdata_RecordInt(RecordRef, "Location.ServingCellInfo.Signal", v->servingCellInfo.signal, timestamp);
        if (result != LE_OK)
        {
            LE_ERROR("Couldn't record location signal reading - %s", LE_RESULT_TXT(result));
            goto done;
        }
    }

    while (i < v->numPciCells) 
    {
        if (strlen(v->pciCellInfo[i].mcc) > 0) 
        {
            snprintf(node, sizeof(node), "Location.PciCellInfo.%d.MCC", i + 1);
            result = le_avdata_RecordString(RecordRef, node, v->pciCellInfo[i].mcc, timestamp);
            if (result != LE_OK)
            {
                LE_ERROR("Couldn't record location cell ID reading - %s", LE_RESULT_TXT(result));
                goto done;
            }
        }

        if (strlen(v->pciCellInfo[i].mnc) > 0) 
        {
            snprintf(node, sizeof(node), "Location.PciCellInfo.%d.MNC", i + 1);
            result = le_avdata_RecordString(RecordRef, node, v->pciCellInfo[i].mnc, timestamp);
            if (result != LE_OK)
            {
                LE_ERROR("Couldn't record location cell ID reading - %s", LE_RESULT_TXT(result));
                goto done;
            }
        }

	if (v->pciCellInfo[i].cid != -1)
	{
	    snprintf(node, sizeof(node), "Location.PciCellInfo.%d.Cid", i + 1);
            LE_INFO("node('%s')", node);
            result = le_avdata_RecordInt(RecordRef, node, v->pciCellInfo[i].cid, timestamp);
            if (result != LE_OK)
            {
                LE_ERROR("Couldn't record location cell ID reading - %s", LE_RESULT_TXT(result));
                goto done;
            }
	}

	i++;
    }

done:
    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * Copies a struct Location between two void pointers
 */
//--------------------------------------------------------------------------------------------------
static void LocationCopyValue
(
    void *dest,      ///< copy destination
    const void *src  ///< copy source
)
{
    struct Location *d = dest;
    const struct Location *s = src;
    uint32_t i = 0;

    
    d->numPciCells = s->numPciCells;
    while (i < d->numPciCells)
    {
        strncpy(d->pciCellInfo[i].mcc, s->pciCellInfo[i].mcc, sizeof(d->pciCellInfo[i].mcc));
        strncpy(d->pciCellInfo[i].mnc, s->pciCellInfo[i].mnc, sizeof(d->pciCellInfo[i].mnc));
        d->pciCellInfo[i].cid = s->pciCellInfo[i].cid;
        i++;
    }
}

#ifdef GPS_ENABLE
//--------------------------------------------------------------------------------------------------
/**
 * Read the GPS location
 *
 * @return
 *      LE_OK on success.  Any other return value is a failure.
 */
//--------------------------------------------------------------------------------------------------
static le_result_t GpsRead
(
    void *value  ///< Pointer to a struct Location3d to store the reading in
)
{
    struct Location3d *v = value;
    return mangOH_ReadGps(&v->latitude, &v->longitude, &v->hAccuracy, &v->altitude, &v->vAccuracy);
}

//--------------------------------------------------------------------------------------------------
/**
 * Checks to see if the location has changed sufficiently to warrant recording of a new reading.
 *
 * @return
 *      true if the threshold for recording has been exceeded
 */
//--------------------------------------------------------------------------------------------------
static bool GpsThreshold
(
    const void *recordedValue, ///< Last recorded angular velocity
    const void *readValue      ///< Most recent angular velocity
)
{
    const struct Location3d *v1 = recordedValue;
    const struct Location3d *v2 = readValue;

    double deltaLat = v2->latitude - v1->latitude;
    double deltaLon = v2->longitude - v1->longitude;
    /*
    double deltaHAccuracy = v2->hAccuracy - v1->hAccuracy;
    double deltaAltitude = v2->altitude - v1->altitude;
    double deltaVAccuracy = v2->vAccuracy - v1->vAccuracy;
    */

    // TODO: It makes sense to publish a new value if the possible position of the device has
    // changed by certain number of meters. Calculating the number of meters between two latitudes
    // or two longitudes requires complicated math.  Just doing something dumb for now.

    return fabs(deltaLat) + fabs(deltaLon) > 0.01;
}

//--------------------------------------------------------------------------------------------------
/**
 * Records a GPS reading at the given time into the given record
 *
 * @return
 *      - LE_OK on success
 *      - LE_OVERFLOW if the record is full
 *      - LE_FAULT non-specific failure
 */
//--------------------------------------------------------------------------------------------------
static le_result_t GpsRecord
(
    le_avdata_RecordRef_t ref, ///< Record reference to record the value into
    uint64_t timestamp,        ///< Timestamp to associate with the value
    void *value                ///< The struct Gps value to record
)
{
    char path[128] = "lwm2m.6.0.";
    int end = strnlen(path, sizeof(path));
    struct Location3d *v = value;
    le_result_t result = LE_FAULT;

    strcpy(&path[end], "0");
    result = le_avdata_RecordFloat(RecordRef, path, v->latitude, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gps latitude reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    strcpy(&path[end], "1");
    result = le_avdata_RecordFloat(RecordRef, path, v->longitude, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gps longitude reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    strcpy(&path[end], "3");
    result = le_avdata_RecordFloat(RecordRef, path, v->hAccuracy, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gps horizontal accuracy reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    strcpy(&path[end], "2");
    result = le_avdata_RecordFloat(RecordRef, path, v->altitude, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gps altitude reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

    strcpy(&path[end], "MangOH.Sensors.Gps.VerticalAccuracy");
    result = le_avdata_RecordFloat(RecordRef, path, v->vAccuracy, timestamp);
    if (result != LE_OK)
    {
        LE_ERROR("Couldn't record gps vertical accuracy reading - %s", LE_RESULT_TXT(result));
        goto done;
    }

done:
    return result;
}

//--------------------------------------------------------------------------------------------------
/**
 * Copies a struct Location3d between two void pointers
 */
//--------------------------------------------------------------------------------------------------
static void GpsCopyValue
(
    void *dest,      ///< copy destination
    const void *src  ///< copy source
)
{
    struct Location3d *d = dest;
    const struct Location3d *s = src;
    d->latitude = s->latitude;
    d->longitude = s->longitude;
    d->hAccuracy = s->hAccuracy;
    d->altitude = s->altitude;
    d->vAccuracy = s->vAccuracy;
}
#endif // GPS_ENABLE

//--------------------------------------------------------------------------------------------------
/**
 * Handle changes in the AirVantage session state
 *
 * When the session is started the timer to sample the sensors is started and when the session is
 * stopped so is the timer.
 */
//--------------------------------------------------------------------------------------------------
static void AvSessionStateHandler
(
    le_avdata_SessionState_t state,
    void *context
)
{
    switch (state)
    {
        case LE_AVDATA_SESSION_STARTED:
        {
            // TODO: checking for LE_BUSY is a temporary workaround for the session state problem
            // described below.
            LE_DEBUG("Session Started");
            le_result_t status = le_timer_Start(SampleTimer);
            if (status == LE_BUSY)
            {
                LE_INFO("Received session started when timer was already running");
            }
            else
            {
                LE_ASSERT_OK(status);
            }
            break;
        }

        case LE_AVDATA_SESSION_STOPPED:
        {
            LE_DEBUG("Session Stopped");
            le_result_t status = le_timer_Stop(SampleTimer);
            if (status != LE_OK)
            {
                LE_DEBUG("Record push timer not running");
            }

            break;
        }

        default:
            LE_ERROR("Unsupported AV session state %d", state);
            break;
    }
}

COMPONENT_INIT
{
    RecordRef = le_avdata_CreateRecord();

    SampleTimer = le_timer_Create("Location Read");
    LE_ASSERT_OK(le_timer_SetMsInterval(SampleTimer, DelayBetweenReadings * 1000));
    LE_ASSERT_OK(le_timer_SetRepeat(SampleTimer, 0));
    LE_ASSERT_OK(le_timer_SetHandler(SampleTimer, SampleTimerHandler));

    HandlerRef = le_avdata_AddSessionStateHandler(AvSessionStateHandler, NULL);
    AvSession = le_avdata_RequestSession();
    LE_FATAL_IF(AvSession == NULL, "Failed to request avdata session");
}
