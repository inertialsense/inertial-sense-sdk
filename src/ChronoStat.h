/**
 * @file PeriodStat.h 
 * @brief ${BRIEF_DESC}
 *
 * @author Kyle Mallory on 9/17/25.
 * @copyright Copyright (c) 2025 Inertial Sense, Inc. All rights reserved.
 */

#ifndef IS_SDK__CHRONO_STAT_H
#define IS_SDK__CHRONO_STAT_H

#include <string>
#include <chrono>
#include <cmath>

#include "util/util.h"
#include "core/msg_logger.h"

/**
 * A utility class which calculates a number of time-based measurements from 2 or more samples over time.
 * ChronoStat can be used with a local/internal clock, if sample() is called without any parameters. Or
 * it can be used with an external clock, by passing a representation of time, as a double expressed as
 * seconds from some known reference time.  In otherwords, this class doesn't really care about "time"
 * per se, rather more then differences between the values passed into sample().  It does however,
 * maintain a local clock reference used to note the time when sample() was called, which is used to
 * when calling lastLocalTs() and lastSampleAgeMs()
 */
class ChronoStat {
private:
    constexpr static double INVALID_DDT_MIN_STAT = 99999.0;
    int cnt = 0;
    double timeLast = NAN;                  //!< the last time that a sample was taken
    std::chrono::high_resolution_clock::time_point localTimeTs;     // this should initialize to the clocks epoch
    std::string label;

public:
    double dt = 0;                          //!< the delta in time between the previous 2 samples, in seconds,
    double dtMin = INVALID_DDT_MIN_STAT;    //!< the lowest dt from all samples, in seconds
    double dtMax = -INVALID_DDT_MIN_STAT;   //!< the largest dt from all samples, in seconds
    double dtMinTime = 0;                   //!< the sample time of the lowest dt
    double dtMaxTime = 0;                   //!< the sample time of the largest dt
    double dtAvg = 0;                       //!< the average dt across all samples (in seconds)
    int dtCnt = 0;                          //!< the number of dt samples (should always be cnt - 1)
    double duration = 0;                    //!< the sum of all dt's - effectively the total time between the first and last sample.

    double dtLast = NAN;                    //!< timestamp of the last dt sample, used for ddt calculations
    double ddt = 0;
    double ddtMin = INVALID_DDT_MIN_STAT;
    double ddtMax = -INVALID_DDT_MIN_STAT;
    double ddtMinTime = 0;
    double ddtMaxTime = 0;
    double ddtAvg = 0;
    int ddtCnt = 0;

    double rate = 0;                //!< the rate/second of samples
    uint64_t accrual = 0;           //!< a general purpose, user counter - call accrue()
    double accrualRate = 0;         //!< the rate/second of the accrual counter

    /**
     * @returns the number of times this stat has been sampled
     */
    inline int count() { return cnt; }

    /**
     * @returns true indicating that more than one sample has been taken and that stats are available, otherwise false
     */
    inline bool hasData() { return cnt > 1; }

    inline double lastSampleTime() { return timeLast; }

    /**
     * @returns the current timestamp (chrono::time_point) of the most recent sample.
     *   If sample has never been called, this should return the Epoch of the source
     *   (std::chrono::hish_resolution_clock) clock, which is typically Jan 1, 1970.
     */
    inline std::chrono::high_resolution_clock::time_point lastLocalTs() { return localTimeTs; }

    /**
     * @returns a timestamp in milliseconds at the time the last sample was taken.
     * NOTE: this is a milliseconds since epoch, cast to a uint32_t which will truncate the upper bits and thus
     *   does not represent an actual wall-clock/system time, as it will rollover approximately every 49 days
     */
    inline uint32_t lastLocalTsMs() {
        if (cnt < 1) return 0;
        auto duration = localTimeTs.time_since_epoch(); // Get the duration since the epoch
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
    }

    /**
     * @returns the time elapsed (milliseconds) since the last sample (derived from the local system clock).
     * NOTE that this uses the local time when the sample was made, and is independent of the time provided
     * to the call to sample() (if any). If there is no previous sample, returns UINT32_MAX.
     */
    inline uint32_t lastSampleAgeMs() {
        if (cnt < 1) return UINT32_MAX;
        auto now = std::chrono::high_resolution_clock::now();
        auto durationMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - localTimeTs); // Get the duration since the epoch
        return static_cast<uint32_t>(durationMs.count());
    }

    /**
     * Assigns a label to this stat instance; has no bearing other than convenience to toString()
     * @param newLabel the new label/name to assign to this instance.
     */
    void setLabel(const std::string& newLabel) { label = newLabel; }

    /**
     * @returns the current label associated with this stat instance.
     */
    std::string getLabel() { return label; }

    /**
     * @returns true if a label has been assigned, otherise false.
     */
    bool hasLabel() { return !label.empty(); }

    /**
     * @brief clears all current values and re-initializes the stat.
     */
    void clear() {
        cnt = 0;
        timeLast = NAN;
        localTimeTs = (std::chrono::high_resolution_clock::time_point::min)();     // this should initialize to the clocks epoch, and ALSO make Windows min/max macros happy.

        dt = 0;
        dtMin = INVALID_DDT_MIN_STAT;
        dtMax = -INVALID_DDT_MIN_STAT;
        dtMinTime = 0;
        dtMaxTime = 0;
        dtAvg = 0;
        dtCnt = 0;
        duration = 0;

        dtLast = NAN;
        ddt = 0;
        ddtMin = INVALID_DDT_MIN_STAT;
        ddtMax = -INVALID_DDT_MIN_STAT;
        ddtMinTime = 0;
        ddtMaxTime = 0;
        ddtAvg = 0;
        ddtCnt = 0;

        rate = 0;
        accrual = 0;
        accrualRate = 0;
    }

    /**
     * @brief samples the specified time (or current clock time as seconds since epoch if not specified) and updates stats
     * @param time a numberical representation of some time, as seconds.
     */
    void sample(double time = NAN) {
        localTimeTs = std::chrono::high_resolution_clock::now();
        if (std::isnan(time)) {      // == FP_NAN
            auto duration_since_epoch = localTimeTs.time_since_epoch(); // Get the duration since the epoch
            std::chrono::duration<double> seconds_double = duration_since_epoch;    // Convert the duration to a duration with a double representation for seconds
            time = seconds_double.count();                                          // Get the count of seconds as a double
        }

        log_debug(IS_LOG_CHRONO_STATS, "%-20s ts:%6.3f :: ", label.c_str(), time);

        cnt++;
        if (std::isnan(timeLast))
        {   // First sample
            dtMin = INVALID_DDT_MIN_STAT;
            dtMax = -INVALID_DDT_MIN_STAT;
        }
        else
        {
            dt = time - timeLast;
            double alpha = 1.0 / (1.0 + dtCnt);
            double beta = 1.0 - alpha;
            dtAvg = beta * dtAvg + alpha * dt;
            dtCnt++;
            duration += dt;

            log_debug(IS_LOG_CHRONO_STATS, "dt %.4f  ", dt);
            log_debug(IS_LOG_CHRONO_STATS, "avg %.4f  ", dtAvg);

            if (dt < dtMin) { dtMin = dt;  dtMinTime = time; log_debug(IS_LOG_CHRONO_STATS, "dtMin %.3f  ", dtMin); }
            if (dt > dtMax) { dtMax = dt;  dtMaxTime = time; log_debug(IS_LOG_CHRONO_STATS, "dtMax %.3f  ", dtMax); }

            if (std::isnan(dtLast)) {   // First sample
                ddtMin = INVALID_DDT_MIN_STAT;
                dtMax = -INVALID_DDT_MIN_STAT;
            } else {
                ddt = dt - dtLast;
                double alpha = 1.0 / (1.0 + ddtCnt);
                double beta = 1.0 - alpha;
                ddtAvg = beta * ddtAvg + alpha * ddt;
                ddtCnt++;

                if (ddt < ddtMin) { ddtMin = ddt,  ddtMinTime = time; log_debug(IS_LOG_CHRONO_STATS, "dtMin %.3f  ", dtMin); }
                if (ddt > ddtMax) { ddtMax = ddt,  ddtMaxTime = time; log_debug(IS_LOG_CHRONO_STATS, "dtMax %.3f  ", dtMax); }
            }

            rate = (1.0 / dt);
            accrualRate = accrual / rate;
            dtLast = dt;
        }
        timeLast = time;
        log_debug(IS_LOG_CHRONO_STATS, "\n");
    };


    /**
     * @brief a utility function to generate a summary of the stat values
     * @param multiline if true, will generate a multi-line string with the second line contianing ddt stats.
     * @return a string summarizing the values managed by this stat.
     */
    std::string toString(bool multiline = false) {
        if (!hasData())
            return "  !! Insufficient number of samples to determine statistics.";

        std::string out = utils::string_format("  dt: avg %5.1f ms, min %5.1f ms, max %5.1f ms (period: %.3fs %4d smpls)", dtAvg * 1.0e3, dtMin * 1.0e3, dtMax * 1.0e3, duration, cnt);
        if (multiline) {
            out += utils::string_format("\n ddt: avg %5.1f ms, min %5.1f ms, max %5.1f ms", ddtAvg * 1.0e3, ddtMin * 1.0e3, ddtMax * 1.0e3);
        }
        return out;
    };

};


#endif //IS_SDK__CHRONO_STAT_H
