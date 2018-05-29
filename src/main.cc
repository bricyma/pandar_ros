#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rawdata.h"
#include <pandar_msgs/PandarPacket.h>
#include <pandar_msgs/PandarScan.h>

#include "hesaiLidarSDK.h"
using namespace std;

class HesaiLidarClient
{
public:
  HesaiLidarClient(ros::NodeHandle node, ros::NodeHandle nh)
  {
    lidarPublisher = node.advertise<sensor_msgs::PointCloud2>("pandar_points", 10);
    pktsPublisher = node.advertise<pandar_msgs::PandarScan>("pandar_packets", 10);
    
    string serverIp;
    int serverPort;
    string calibrationFile;
    int lidarRecvPort;
    int gpsPort;
    double startAngle;
    string lidarCorrectionFile;
    int laserReturnType;
		int laserCount;
		int pclDataType;
    string pcapFile;
    image_transport::ImageTransport it(nh);

    nh.getParam("mode", mode);
    nh.getParam("pcap_file", pcapFile);
    nh.getParam("server_ip", serverIp);
    nh.getParam("server_port", serverPort);
    nh.getParam("calibration_file", calibrationFile);
    nh.getParam("lidar_recv_port", lidarRecvPort);
    nh.getParam("gps_port", gpsPort);
    nh.getParam("start_angle", startAngle);
    nh.getParam("lidar_correction_file", lidarCorrectionFile);
    nh.getParam("laser_return_type", laserReturnType);
    nh.getParam("laser_count", laserCount);
    nh.getParam("pcldata_type", pclDataType);

    // pcapFile = "/home/pandora/Desktop/pandar40p.pcap";
    
    // online
    if(!mode && serverIp.empty())
    {
      
      hsdk = new HesaiLidarSDK(lidarRecvPort, gpsPort, lidarCorrectionFile,
                      boost::bind(&HesaiLidarClient::lidarCallback, this, _1, _2, _3),
                      NULL, (HesaiLidarRawDataSturct)laserReturnType, laserCount, (HesaiLidarPCLDataType)pclDataType);

      hsdk->start();
    }else{
      // offline
      data_.reset(new pandar_rawdata::RawData(lidarCorrectionFile, laserReturnType, laserCount, pclDataType));
    }

  }

  void cameraCallback(boost::shared_ptr<cv::Mat> matp, double timestamp, int pic_id)
  {

    sensor_msgs::ImagePtr imgMsg;

    switch (pic_id)
    {
    case 0:
      imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *matp).toImageMsg();
      break;
    case 1:
    case 2:
    case 3:
    case 4:
      imgMsg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", *matp).toImageMsg();
      break;
    default:
      ROS_INFO("picid wrong in getImageToPub");
      return;
    }
    imgMsg->header.stamp = ros::Time(timestamp);
    imgPublishers[pic_id].publish(imgMsg);
  }

  void lidarCallback(std::list<PandarPacket> pkts, boost::shared_ptr<PPointCloud> cld, double timestamp)
  {
    publishPackets(pkts);
    pcl_conversions::toPCL(ros::Time(timestamp), cld->header.stamp);
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg(*cld, output);
    lidarPublisher.publish(output);
  }

  // publish ros topic: /pandar_packet
  void publishPackets(std::list<PandarPacket> pkts)
  {
    pandar_msgs::PandarScanPtr scan(new pandar_msgs::PandarScan);
    scan->packets.resize(pkts.size());
    int i = 0;
    for (std::list<PandarPacket>::iterator it = pkts.begin();  it != pkts.end(); ++it){
      for (int j = 0; j < it->size; j++)
        scan->packets[i].data[j] = it->data[j];
      i++; 
    }
    // printf("%d\n", i);  // TODO, pkt size
    scan->header.stamp = ros::Time::now();
    pktsPublisher.publish(scan);

  }
 
  /** packets => points
   *  two modes: online, offline 
   *  scanMsg => lidarPacketList
   *  lidarPacketList => outMsg
   *  outMsg => rosMsg
   */
  void pktCallback(const pandar_msgs::PandarScan::ConstPtr &scanMsg){
    boost::shared_ptr<PPointCloud> outMsg(new PPointCloud());
    outMsg->header.frame_id = "pandar";
    outMsg->height = 1;
    
    int lidarRotationStartAngle = 0;
    time_t gps1;
    double timestamp;
    GPS_STRUCT_T gps2;
    gps1 = 0;
    gps2.gps = 0;
    gps2.used = 1;
    gps2.usedHour = 1;

    for (int i=0; i<scanMsg->packets.size(); i++){
      PandarPacket pkt;
      for (int j = 0; j < 1500; j++){
        pkt.data[j] = scanMsg->packets[i].data[j];
      }
      pkt.size = 1256;      
      int ret = data_->unpack(pkt, *outMsg, gps1, gps2, lidarRotationStartAngle);
    }
    if(outMsg->points.size() > 0) {
      timestamp = outMsg->points[0].timestamp;
      pcl_conversions::toPCL(ros::Time(timestamp), outMsg->header.stamp);
      sensor_msgs::PointCloud2 output;
      pcl::toROSMsg(*outMsg, output);
      lidarPublisher.publish(output);
    }
  }


  int mode;  // online: 0; offline: 1
private:
  ros::Publisher lidarPublisher, pktsPublisher;
  image_transport::Publisher imgPublishers[5];
  HesaiLidarSDK* hsdk;
  boost::shared_ptr<pandar_rawdata::RawData> data_;
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "pandora_ros");
  ros::NodeHandle nh("~");
  ros::NodeHandle node;
  HesaiLidarClient pandoraClient(node, nh);
  ros::Subscriber pktsSubscriber;
  if(pandoraClient.mode) 
    pktsSubscriber = node.subscribe("/pandar_packets", 10, &HesaiLidarClient::pktCallback, &pandoraClient);
  ros::spin();
  return 0;
}
