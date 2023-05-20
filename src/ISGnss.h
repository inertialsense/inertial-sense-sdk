
#ifndef IS_GNSS_H_
#define IS_GNSS_H_



std::string gnssIdToGnssName(int gnssId);
char gnssIdToGnssPrefix(int gnssId);
std::string gnssIdSigIdToSignalName(int gnssId, int sigId);


#endif //IS_GNSS_H_
