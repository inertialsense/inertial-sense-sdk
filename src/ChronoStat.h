/**
 * @file ChronoStat.h
 * @brief Chronometry/timing-statistics helper: given a stream of sample() calls (either
 *        externally-timestamped or using an internal clock), incrementally computes the
 *        inter-sample interval (dt) and its min/max/average/variance, the second-order
 *        variation between successive intervals (ddt), the resulting sample rate, and a
 *        user-driven accrual counter/rate -- all via Welford's online algorithm so no history
 *        of samples needs to be retained.
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

// #define ADDITIONAL_DEBUGGING     // Use this if you are troubleshooting the ChronoStat calculations in sample();

#ifdef ADDITIONAL_DEBUGGING
/** Begins building a per-sample debug log line (label + timestamp prefix); active only when ADDITIONAL_DEBUGGING is defined. */
#define START_DEBUG_MSG()           std::string logMsg = utils::string_format("%-20s ts:%6.3f :: ", label.c_str(), time)
/** Appends a printf-style-formatted fragment to the in-progress debug log line started by START_DEBUG_MSG(). @param ... a printf-style format string and its arguments */
#define APPEND_DEBUG_MSG(...)       logMsg += utils::string_format(__VA_ARGS__)
/** Emits the debug log line built by START_DEBUG_MSG()/APPEND_DEBUG_MSG() via the given log macro. @param msg_level the log_* macro (e.g. log_debug) to emit the line with */
#define END_DEBUG_MSG(msg_level)    msg_level(IS_LOG_CHRONO_STATS, "%s", logMsg.c_str())
#else
/** No-op when ADDITIONAL_DEBUGGING is not defined. */
#define START_DEBUG_MSG()           {}
/** No-op when ADDITIONAL_DEBUGGING is not defined. @param ... unused */
#define APPEND_DEBUG_MSG(...)       {}
/** No-op when ADDITIONAL_DEBUGGING is not defined. @param msg_level unused */
#define END_DEBUG_MSG(msg_level)    {}
#endif


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
    std::chrono::high_resolution_clock::time_point localTimeTs;     //!< local-clock timestamp of the most recent sample; default-initializes to the clock's epoch
    std::string label;                      //!< optional label/name for this stat instance, used only by toString()
    double dtM2 = 0;                        //!< Welford's algorithm running sum of squared differences from the mean, used to derive dtVariance

public:
    double dt = 0;                          //!< the delta in time between the previous 2 samples, in seconds,
    double dtMin = 0;                       //!< the lowest dt from all samples, in seconds (0 until first dt)
    double dtMax = 0;                       //!< the largest dt from all samples, in seconds (0 until first dt)
    double dtMinTime = 0;                   //!< the sample time of the lowest dt
    double dtMaxTime = 0;                   //!< the sample time of the largest dt
    double dtAvg = 0;                       //!< the average dt across all samples (in seconds)
    double dtVariance = 0;                  //!< the (population) variance of dt across all samples, computed online via Welford's algorithm
    int dtCnt = 0;                          //!< the number of dt samples (should always be cnt - 1)
    double duration = 0;                    //!< the sum of all dt's - effectively the total time between the first and last sample.

    double dtLast = NAN;                    //!< timestamp of the last dt sample, used for ddt calculations
    double ddt = 0;                         //!< the delta between the two most recent dt values (the "second derivative" of the sample times), in seconds
    double ddtMin = 0;                      //!< the lowest ddt from all samples, in seconds (0 until first ddt)
    double ddtMax = 0;                      //!< the largest ddt from all samples, in seconds (0 until first ddt)
    double ddtMinTime = 0;                  //!< the sample time of the lowest ddt
    double ddtMaxTime = 0;                  //!< the sample time of the largest ddt
    double ddtAvg = 0;                      //!< the average ddt across all samples (in seconds)
    int ddtCnt = 0;                         //!< the number of ddt samples (should always be dtCnt - 1)

    double rate = 0;                //!< the rate/second of samples
    uint64_t accrual = 0;           //!< a general purpose, user counter - call accrue()
    double accrualRate = 0;         //!< the rate/second of the accrual counter

    /**
     * @return the number of times this stat has been sampled
     */
    inline int count() { return cnt; }

    /**
     * @return true indicating that more than one sample has been taken and that stats are available, otherwise false
     */
    inline bool hasData() { return cnt > 1; }

    /**
     * @return the value of time (either externally supplied or from the local clock) passed to the most recent call to sample(), or NAN if sample() has never been called.
     */
    inline double lastSampleTime() { return timeLast; }

    /**
     * @return the current timestamp (chrono::time_point) of the most recent sample.
     *   If sample has never been called, this should return the Epoch of the source
     *   (std::chrono::high_resolution_clock) clock, which is typically Jan 1, 1970.
     */
    inline std::chrono::high_resolution_clock::time_point lastLocalTs() { return localTimeTs; }

    /**
     * @return a timestamp in milliseconds at the time the last sample was taken.
     * NOTE: this is a milliseconds since epoch, cast to a uint32_t which will truncate the upper bits and thus
     *   does not represent an actual wall-clock/system time, as it will rollover approximately every 49 days
     */
    inline uint32_t lastLocalTsMs() {
        if (cnt < 1) return 0;
        auto dur = localTimeTs.time_since_epoch(); // Get the duration since the epoch
        return static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::milliseconds>(dur).count());
    }

    /**
     * @return the time elapsed (milliseconds) since the last sample (derived from the local system clock).
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
     * @return the current label associated with this stat instance.
     */
    std::string getLabel() { return label; }

    /**
     * @return true if a label has been assigned, otherise false.
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
        dtMin = 0;
        dtMax = 0;
        dtMinTime = 0;
        dtMaxTime = 0;
        dtAvg = 0;
        dtVariance = 0;
        dtM2 = 0;
        dtCnt = 0;
        duration = 0;

        dtLast = NAN;
        ddt = 0;
        ddtMin = 0;
        ddtMax = 0;
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

        START_DEBUG_MSG();

        cnt++;
        if (std::isnan(timeLast))
        {   // First sample -- no dt yet; leave dt stats at 0
            dtMin = 0;
            dtMax = 0;
        }
        else
        {
            dt = time - timeLast;
            double alpha = 1.0 / (1.0 + dtCnt);
            double beta = 1.0 - alpha;
            double dtDelta = dt - dtAvg;              // dtAvg is still the mean of the previous dtCnt samples here
            dtAvg = beta * dtAvg + alpha * dt;
            double dtDelta2 = dt - dtAvg;              // dtAvg is now the updated mean
            dtM2 += dtDelta * dtDelta2;                 // Welford's algorithm running sum of squares
            dtVariance = dtM2 / (dtCnt + 1);            // population variance over all dt samples so far
            dtCnt++;
            duration += dt;

            APPEND_DEBUG_MSG("dt %.4f  ", dt);
            APPEND_DEBUG_MSG("avg %.4f  ", dtAvg);

            if (dtCnt == 1 || dt < dtMin) { dtMin = dt;  dtMinTime = time; APPEND_DEBUG_MSG("dtMin %.3f  ", dtMin); }
            if (dtCnt == 1 || dt > dtMax) { dtMax = dt;  dtMaxTime = time; APPEND_DEBUG_MSG("dtMax %.3f  ", dtMax); }

            if (std::isnan(dtLast)) {   // First dt sample -- no ddt yet; leave ddt stats at 0 (NOT dtMax, which was just set above)
                ddtMin = 0;
                ddtMax = 0;
            } else {
                ddt = dt - dtLast;
                double alphaLocal = 1.0 / (1.0 + ddtCnt);
                double betaLocal = 1.0 - alphaLocal;
                ddtAvg = betaLocal * ddtAvg + alphaLocal * ddt;
                ddtCnt++;

                if (ddtCnt == 1 || ddt < ddtMin) { ddtMin = ddt,  ddtMinTime = time; APPEND_DEBUG_MSG("ddtMin %.3f  ", ddtMin); }
                if (ddtCnt == 1 || ddt > ddtMax) { ddtMax = ddt,  ddtMaxTime = time; APPEND_DEBUG_MSG("ddtMax %.3f  ", ddtMax); }
            }

            rate = (dt != 0) ? (1.0 / dt) : 0;
            accrualRate = (rate != 0) ? ((double)accrual / rate) : 0;
            dtLast = dt;
        }
        timeLast = time;
        END_DEBUG_MSG(log_more_debug);
    };


    /**
     * @return the (population) standard deviation of dt across all samples, in the same units as dt.
     */
    inline double dtStdDev() { return std::sqrt(dtVariance); }

    /**
     * @brief a utility function to generate a summary of the stat values
     * @param multiline if true, will generate a multi-line string with the second line containing ddt stats.
     * @return a string summarizing the values managed by this stat.
     */
    std::string toString(bool multiline = false) {
        std::string out = getLabel() + " ::";
        if (!hasData()) {
            out += "  !! Insufficient number of samples to determine statistics.";
        } else {
            out += utils::string_format("  dt: avg %5.1f ms, min %5.1f ms, max %5.1f ms (period: %.3fs %4d smpls)", dtAvg * 1.0e3, dtMin * 1.0e3, dtMax * 1.0e3, duration, cnt);
            if (multiline) {
                out += utils::string_format("\n ddt: avg %5.1f ms, min %5.1f ms, max %5.1f ms", ddtAvg * 1.0e3, ddtMin * 1.0e3, ddtMax * 1.0e3);
            }
        }
        return out;
    };

};


#endif //IS_SDK__CHRONO_STAT_H
