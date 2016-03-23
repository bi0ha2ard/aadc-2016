#ifndef _SR_LOGGER_H_
#define _SR_LOGGER_H_

#include <fstream>
#include <ios>
#include <adtf_platform_inc.h>
#include <adtf_plugin_sdk.h>
#include <adtf_graphics.h>
#include <adtf_utils.h>

using namespace std;

class SR_Logger
{
public:
	SR_Logger(cString vName, cString path = "../aadc_log");
	~SR_Logger();
	void GLogTrap(cString vData,tTimeStamp vTimeStamp,adtf_util::cDateTime vDateTime = adtf_util::cDateTime::GetCurrentDateTime());
	void GLogTrap(cString vData);

	/**
	 * @brief log Usage: logger.log() << "foo" << 123 << 12.323 << std::endl;
	 * @return
	 */
	std::ofstream &log();
private:
	std::ofstream file;
	cFilename fname;
	cDateTime startTime;

};
#endif
