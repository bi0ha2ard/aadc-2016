#ifndef MAP_DATA_H_
#define MAP_DATA_H_
#include <stdint.h>

#define MAX_CROSSING_DISTANCE 2.5 //used to decide if crossing is close enough to trigger a stop maneuver
#define MIN_CROSSING_DETECTIONS 1 //used to decide if crossing is reliable enough detected to trigger a stop maneuver

#include <opencv/cv.h>
#include <roadsigns.h>

typedef struct {
	double pixelCenter;
	double meterCenter;
	double meterPerPixel;
} ImageToCarMap;

enum TileType {
	Tile_Unknown,
	Free,
	Straight,
	Crossing_T,
	Crossing_X,
	Curve_Small,
	Curve_Large,
	Curve_S_Right,
	Curve_S_Left,
	Parking_Across,
	Parking_Along
};

enum TileOrientation {
	// For T Crossings and parking spaces, Direction of the T Leg relative to car
	Orientation_N,
	Orientation_S,
	Orientation_E,
	Orientation_W,
	// For curves
	Orientation_Left,
	Orientation_Right,
};

typedef struct {
	TileType type;
	TileOrientation orientation;
	double centerXPixel;
	double centerYPixel;
} MapTileMsg;

typedef struct {
	cv::Point2d center;
	cv::Point2d orientationVector;
	int numDetections;
	uint8_t exits;
	bool stopLine;
} CrossingDetectionEvent;

typedef struct {
	std::vector<cv::Point2d> leftLane;
	std::vector<cv::Point2d> rightLane;
	std::vector<cv::Point2d> centerLane;
} LaneData;

typedef struct {
	cv::Point3d pos;
	RoadSign id;
        float theta;
} SignStruct;

enum ParkingType {
	ParkingTypeCross,
	ParkingTypeParallel
};

typedef struct{
	bool straight_lane;
	bool right_lane;
        bool left_lane;
} BlockedSpots;

typedef struct{
	double dist;
	ParkingType type;  // 0 crossing, 1 parallel
} ParkingSearchRequest;

#define HAS_EXIT_RIGHT(crossing) (((crossing).exits >> 2) & 1)
#define HAS_EXIT_STRAIGHT(crossing) (((crossing).exits >> 3) & 1)
#define HAS_EXIT_LEFT(crossing) (((crossing).exits >> 0) & 1)
#define HAS_EXIT_BACK(crossing) (((crossing).exits >> 1) & 1)

#define EXIT_NORTH 0b00001000
#define EXIT_EAST  0b00000100
#define EXIT_SOUTH  0b00000010
#define EXIT_WEST   0b00000001
#define EXIT_RIGHT EXIT_EAST
#define EXIT_STRAIGHT EXIT_NORTH
#define EXIT_LEFT EXIT_WEST


#define MEDIA_TYPE_MAPDATA                          0x00007331
#define MEDIA_SUBTYPE_MAPDATA_IMAGE_TO_CAR_MAP      0x00000001
#define MEDIA_SUBTYPE_MAPDATA_MAP_TILE_MSG          0x00000002
#define MEDIA_SUBTYPE_MAPDATA_CROSSING_EVENT        0x00000003
#define MEDIA_SUBTYPE_MAPDATA_SIGN                  0x00001887
#define MEDIA_SUBTYPE_MAPDATA_PARKING_SEARCH_REQ    0x00011887
#define MEDIA_SUBTYPE_MAPDATA_LANEDATA		    0x00000004
#define MEDIA_SUBTYPE_MAPDATA_OBSTACLES	            0x00000006
#define MEDIA_SUBTYPE_MAPDATA_BLOCKEDSPOTS	    0x00111887
#endif
