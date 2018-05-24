

#include "hesaiLidarSDK_IN.h"
#ifdef HESAI_WITH_CAMERA 
#include "utilities.h"

Size HesaiLidarSDK_IMAGE_SIZE(HesaiLidarSDK_IMAGE_WIDTH, HesaiLidarSDK_IMAGE_HEIGHT);
#endif

int HS_L40_GPS_Parse(HS_LIDAR_L40_GPS_Packet *packet, const unsigned char *recvbuf)
{
	int index = 0;
	packet->flag = (recvbuf[index] & 0xff) | ((recvbuf[index + 1] & 0xff) << 8);
	index += HS_LIDAR_L40_GPS_PACKET_FLAG_SIZE;
	packet->year = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_YEAR_SIZE;
	packet->month = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_MONTH_SIZE;
	packet->day = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_DAY_SIZE;
	packet->second = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_SECOND_SIZE;
	packet->minute = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_MINUTE_SIZE;
	packet->hour = (recvbuf[index] & 0xff - 0x30) + (recvbuf[index + 1] & 0xff - 0x30) * 10;
	index += HS_LIDAR_L40_GPS_PACKET_HOUR_SIZE;
	packet->fineTime = (recvbuf[index] & 0xff) | (recvbuf[index + 1] & 0xff) << 8 |
										 ((recvbuf[index + 2] & 0xff) << 16) | ((recvbuf[index + 3] & 0xff) << 24);
#ifdef DEBUG
        printf("error gps\n");
        if(packet->year != 18)
        {
                char str[128];
                int fd = open("/var/tmp/error_gps.txt" , O_RDWR  | O_CREAT , 0666);
                lseek(fd , 0 , SEEK_END);
                int i =0;
                for(i = 0 ; i < 512 ; i ++)
                {
                        sprintf(str , "%02x " , recvbuf[i]);
                        write(fd , str , strlen(str));
                }
                write(fd , "\n" , 1);
                close(fd);
        }
#endif

	return 0;
}


void HesaiLidarSDK_internal::init(
			const unsigned short lidarRecvPort,
			const unsigned short gpsRecvPort,
			const double startAngle,			
			const std::string lidarCorrectionFile,
			boost::function<void(std::list<PandarPacket> pkts, boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
			boost::function<void(unsigned int timestamp)> gpsCallback,
			const unsigned int laserReturnType,
			const unsigned int laserCount,
			const unsigned int pclDataType)
{
	userLidarCallback = lidarCallback;
	userGpsCallback = gpsCallback;
	lidarRotationStartAngle = static_cast<int>(startAngle * 100);

	gps1 = 0;
	gps2.gps = 0;
	gps2.used = 1;
	gps2.usedHour = 1;
	lidarLastGPSHourSecond = 0;
	
	sem_init(&lidarSem, 0, 0);
	
	pthread_mutex_init(&lidarLock, NULL);
	pthread_mutex_init(&lidarGpsLock, NULL);

	lidarRecvThread = 0;
	lidarProcessThread = 0;
	readPacpThread = 0;
	
	

	input.reset(new pandar_pointcloud::Input(lidarRecvPort, gpsRecvPort));
	data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile, laserReturnType, laserCount, pclDataType));
}

HesaiLidarSDK_internal::HesaiLidarSDK_internal(
			const unsigned short lidarRecvPort,
			const unsigned short gpsRecvPort,
			const double startAngle,			
			const std::string lidarCorrectionFile,
			boost::function<void(std::list<PandarPacket> pkts, boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback,
			boost::function<void(unsigned int timestamp)> gpsCallback,
			const unsigned int laserReturnType,
			const unsigned int laserCount,
			const unsigned int pclDataType)

{
	init(
			lidarRecvPort,
			gpsRecvPort,
			startAngle,
			
			lidarCorrectionFile,
			
			lidarCallback,
			gpsCallback,
			laserReturnType, laserCount, pclDataType);
}

HesaiLidarSDK_internal::HesaiLidarSDK_internal(
			boost::function<void(std::list<PandarPacket> pkts, boost::shared_ptr<PPointCloud> cld, double timestamp)> lidarCallback)
{
	init(
			HesaiLidarSDK_DEFAULT_LIDAR_RECV_PORT,
			HesaiLidarSDK_DEFAULT_GPS_RECV_PORT,
			HesaiLidarSDK_DEFAULT_START_ANGLE,
			std::string(""),
			lidarCallback,
			NULL, 0, 40, 0);
}

HesaiLidarSDK_internal::~HesaiLidarSDK_internal()
{
	stop();
}

int HesaiLidarSDK_internal::start()
{
	stop();
	switch (useMode)
	{
	case PandoraUseModeReadPcap:
	{
		// setupReadPcap();
		break;
	}

	case PandoraUseModeOnlyLidar:
	{
		setupLidarClient();
		break;
	}

	default:
	{
		setupLidarClient();
		break;
		// printf("wrong useMode\n");
		// return -1;
	}
	}

	return 0;
}


void HesaiLidarSDK_internal::setupLidarClient()
{
	continueLidarRecvThread = true;
	continueProcessLidarPacket = true;
	lidarProcessThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::processLiarPacket, this));
	lidarRecvThread = new boost::thread(boost::bind(&HesaiLidarSDK_internal::lidarRecvTask, this));
}



void HesaiLidarSDK_internal::stop()
{
	continueLidarRecvThread = false;
	continueProcessLidarPacket = false;
	
	continueReadPcap = false;

	if (lidarProcessThread)
	{
		lidarProcessThread->join();
		delete lidarProcessThread;
		lidarProcessThread = 0;
	}
	if (lidarRecvThread)
	{
		lidarRecvThread->join();
		delete lidarRecvThread;
		lidarRecvThread = 0;
	}

}






void HesaiLidarSDK_internal::lidarRecvTask()
{
	while (continueLidarRecvThread)
	{
		PandarPacket pkt;
		int rc = input->getPacket(&pkt);
		if (rc == -1)
		{
			continue;
		}
		if (rc == 1)
		{
			// gps packet;
			HS_L40_GPS_Parse(&hesaiGps, &pkt.data[0]);
			processGps(hesaiGps);
			continue;
		}
		// printf ("%f\n", pkt.stamp);
		pushLiDARData(pkt);
		// internalAnalysisPacket(pkt);
	}
}

void HesaiLidarSDK_internal::processLiarPacket()
{
	double lastTimestamp = 0.0f;
	struct timespec ts;

	int count = 0;
	int before_count = 0;
	while (continueProcessLidarPacket)
	{
		boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());
		if (clock_gettime(CLOCK_REALTIME, &ts) == -1)
		{
			printf("get time error\n");
		}

		ts.tv_sec += 1;
		if (sem_timedwait(&lidarSem, &ts) == -1)
		{
			continue;
		}
		pthread_mutex_lock(&lidarLock);
		PandarPacket packet = lidarPacketList.front();
		lidarPacketList.pop_front();

		pthread_mutex_unlock(&lidarLock);
		outMsg->header.frame_id = "pandar";
		outMsg->height = 1;

		pthread_mutex_lock(&lidarGpsLock);
		// printf("gps: %d\n", gps1);
		int ret = data_->unpack(packet, *outMsg, gps1, gps2, lidarRotationStartAngle);

		// topic: /pandar_packets
		lidarPackets.push_back(packet);

		pthread_mutex_unlock(&lidarGpsLock);
		if (ret == 1 && outMsg->points.size() > 0)
		{
			lastTimestamp = outMsg->points[0].timestamp;
			if (userLidarCallback){
				userLidarCallback(lidarPackets, outMsg, lastTimestamp);
				lidarPackets.clear();
			}
		}
	}
}

// boost::shared_ptr<PPointCloud> HesaiLidarSDK_internal::processScan(const pandar_msgs::PandarScan::ConstPtr &scanMsg)
// {
    
//     double lastTimestamp = 0.0f;
//     struct timespec ts;

//     int count = 0;
//     int before_count = 0;
//     boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());
      
//     PandarPacket packet;

//     outMsg->header.frame_id = "pandar";
//     outMsg->height = 1;
      
//     for (int i = 0; i < scanMsg->packets.size(); ++i){
//         for (int j = 0; j < scanMsg->packets[i].data.size(); ++j){
//           packet->data[j] = scanMsg->packets[i].data[j];
//         }
//         int ret = data_->unpack(packet, *outMsg, gps1, gps2, lidarRotationStartAngle);
//     }

      
//     if (ret == 1 && outMsg->points.size() > 0)
//     	return outMsg;
// }





void HesaiLidarSDK_internal::pushLiDARData(PandarPacket packet)
{
	pthread_mutex_lock(&lidarLock);
	lidarPacketList.push_back(packet);
	if (lidarPacketList.size() > 6)
	{
		sem_post(&lidarSem);
	}
	pthread_mutex_unlock(&lidarLock);
}

void HesaiLidarSDK_internal::processGps(HS_LIDAR_L40_GPS_Packet &gpsMsg)
{
	struct tm t;
	
	t.tm_sec = 0;
	t.tm_min = 0;

	t.tm_hour = gpsMsg.hour;
	t.tm_mday = gpsMsg.day;
	t.tm_mon = gpsMsg.month - 1;
	t.tm_year = gpsMsg.year + 2000 - 1900;
	t.tm_isdst = 0;

	int curlidarLastGPSHourSecond = mktime(&t);

	t.tm_sec = gpsMsg.second;
	t.tm_min = gpsMsg.minute;

	if (lidarLastGPSSecond != (mktime(&t) + 1))
	{
		// Send the last GPS Time when the PPS occurs
		lidarLastGPSSecond = mktime(&t) + 1;
		pthread_mutex_lock(&lidarGpsLock);
		gps2.gps = lidarLastGPSSecond;
		gps2.used = 0;
		if (curlidarLastGPSHourSecond != lidarLastGPSHourSecond)
		{
			lidarLastGPSHourSecond = curlidarLastGPSHourSecond;
			gps2.usedHour = 0;
		}

		// TODO: if the jump is too big(24 * 3600 = one day) , We regard this GPS is reliable. use it.
		if((curlidarLastGPSHourSecond - lidarLastGPSHourSecond ) > (24 * 3600))
		{
			gps1 = 0;
			gps2.usedHour = 0;
		}

		memcpy(&gps2.t, &t, sizeof(struct tm));
		gps2.t.tm_min = 0;
		gps2.t.tm_sec = 0;

		pthread_mutex_unlock(&lidarGpsLock);
	}
	if (userGpsCallback)
		userGpsCallback(lidarLastGPSSecond);
}


