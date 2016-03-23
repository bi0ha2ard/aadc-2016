#include "SR_Logger.h"

SR_Logger::SR_Logger(cString vName, cString path)
{
	cString myDateTimeInfo=cDateTime::GetCurrentDateTime().Format("%Y%m%d%H%M%S");
	cString myFullPath=path+"/"+vName+".log";
	fname=cFilename(myFullPath);
	file.open(myFullPath.GetPtr(), std::ofstream::out | std::ofstream::app);
	if (!file.good()) {
		LOG_INFO("SR_Logger: Open failed for " + vName);
	}
	startTime = cDateTime::GetCurrentDateTime();
    LOG_INFO("SR_Logger::SR_Logger ["+vName+"] - ["+myDateTimeInfo+"]");
}

SR_Logger::~SR_Logger()
{
	file.close();
}

void SR_Logger::GLogTrap(cString vData, tTimeStamp vTimeStamp, cDateTime vDateTime)
{
	GLogTrap(vData);
}

void SR_Logger::GLogTrap(cString vData)
{
	cDateTime now = adtf_util::cDateTime::GetCurrentDateTime();
	file << now.Format("%m/%d/%Y;%H:%M:%S").GetPtr() << ";" << cString::Format("%010d", static_cast<int>(cDateTime::GetDiff(now, startTime)/1000)).GetPtr() << ";" << vData.GetPtr() << std::endl;
}

ofstream &SR_Logger::log()
{
	cDateTime now = adtf_util::cDateTime::GetCurrentDateTime();
	file << now.Format("%m/%d/%Y;%H:%M:%S").GetPtr() << ";" << cString::Format("%010d", static_cast<int>(cDateTime::GetDiff(now, startTime)/1000)).GetPtr() << ";";
	return file;
}
