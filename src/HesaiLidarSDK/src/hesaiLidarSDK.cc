#include "hesaiLidarSDK.h"
#include "hesaiLidarSDK_IN.h"



HesaiLidarSDK::HesaiLidarSDK(
	boost::function<void(std::list<PandarPacket> pkts, boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	psi = new HesaiLidarSDK_internal(
		lidarCallback);
}

HesaiLidarSDK::HesaiLidarSDK(
	const unsigned short lidarRecvPort,
	const unsigned short gpsRecvPort,
	const std::string lidarCorrectionFile,
	boost::function<void(std::list<PandarPacket> pkts, boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
	boost::function<void(unsigned int timestamp)> gpsCallback,
	const HesaiLidarRawDataSturct laserReturnType,
	const unsigned int laserCount,
	const HesaiLidarPCLDataType pclDataType)
{
	psi = new HesaiLidarSDK_internal(
		lidarRecvPort,
		gpsRecvPort,
		0,
		lidarCorrectionFile,
		lidarCallback,
		gpsCallback,
		(unsigned int)laserReturnType, laserCount, (unsigned int)pclDataType);
}


HesaiLidarSDK::~HesaiLidarSDK()
{
	delete psi;
}

int HesaiLidarSDK::start()
{
	psi->start();
}
void HesaiLidarSDK::stop()
{
	psi->stop();
}
