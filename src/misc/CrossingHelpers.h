#ifndef SR_CROSSING_HELPERS_H_
#define SR_CROSSING_HELPERS_H_

#include <map_data.h>
#include <odometry_data.h>
#include <maneuverdata.h>
#include <vector>


ControllerCommand getCrossingExit(const CrossingDetectionEvent &e, uint8_t exit);

/**
 * @brief getCrossingExit
 * @param e crossing
 * @param m maneuver
 * @param extraOffset move pose further away from center along road
 * @param extraOffset2 move further from center towards outer lane
 * @return The command
 */
ControllerCommand getCrossingExit(const CrossingDetectionEvent &e, ManeuverType m, double extraOffset = 0, double extraOffset2 = 0);
ControllerCommand getCrossingStop(const CrossingDetectionEvent &e, double offs = 0);
void mergeCrossings(std::vector<CrossingDetectionEvent> &merged, CrossingDetectionEvent e, double distThreshold);

#endif
