#ifndef SR_MANEUVER_DATA_H_
#define SR_MANEUVER_DATA_H_

enum ManeuverType {
	ManeuverLeft,
	ManeuverRight,
	ManeuverStraight,
	ManeuverParallelParking,
	ManeuverCrossParking,
	ManeuverPulloutLeft,
	ManeuverPulloutRight,
	ManeuverIdle,			//<! Initial state
	ManeuverComplete 		//<! List of maneuvers is complete, stop and turn on hazard lights
};

enum ManeuverResponse {
	ManeuverResponseComplete,
	ManeuverResponseError
};

enum InitialCarConfiguration {
	InitialConfigurationTrack,
        InitialConfigurationParkedParallel,
	InitialConfigurationParkedCross
};

#endif
