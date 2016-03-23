/**** EulerAngles.c - Convert Euler angles to/from matrix or quat ****/
/* Ken Shoemake, 1993 */
/**
http://www.realtimerendering.com/resources/GraphicsGems/

LICENSE

This code repository predates the concept of Open Source, and predates most licenses along such lines. As such, the official license truly is:

EULA: The Graphics Gems code is copyright-protected. In other words, you cannot claim the text of the code as your own and resell it. Using the code is permitted in any program, product, or library, non-commercial or commercial. Giving credit is not required, though is a nice gesture. The code comes as-is, and if there are any flaws or problems with any Gems code, nobody involved with Gems - authors, editors, publishers, or webmasters - are to be held responsible. Basically, don't be a jerk, and remember that anything free comes with no guarantee.

 **/

/**** QuatTypes.h - Basic type declarations ****/
#ifndef _H_QuatTypes
#define _H_QuatTypes

#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>

/*** Definitions ***/
/*! struct for quaternions */
typedef struct {
	float x;	/**< the x component */
	float y;	/**< the y component */
	float z;	/**< the z component */
	float w;	/**< the w component */
} Quat; 
/*! enum for quaternions */
/**
enum  
{
	X,		
	Y,		
	Z,		
	W		
} QuatPart;
**/
// Replaces enum QuatPart, because of Compiler Errors
#define _X 0
#define _Y 1
#define _Z 2
#define _W 3

/*! Creating struct for received Wheel data */
struct tWheelData
{
	tInt8 i8WheelDir;
	tUInt32 ui32ArduinoTimestamp;
	tUInt32 ui32WheelTach;
	tFloat32 f32Speed;
};

struct tData
{
	tUInt32 ui32ArduinoTimestamp;
	tFloat32 f32Value;
};


typedef float HMatrix[4][4]; /* Right-handed, for column vectors */
typedef Quat EulerAngles;    /* (x,y,z)=ang 1,2,3, w=order code  */
#endif
