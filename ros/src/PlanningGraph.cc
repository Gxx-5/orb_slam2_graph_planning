#include "PlanningGraph.h"
#include <iostream>
#include <vector>
#include <map>
#include "KeyFrame.h"
#include <math.h>

using namespace std;

using namespace ORB_SLAM2;




int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    if(argc > 1) {
        ROS_WARN ("Arguments supplied via command line are neglected.");
    }

    ros::NodeHandle node_handle;

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    image_transport::ImageTransport image_transport (node_handle);
    cout<<"testing3"<<endl;
    PlanningGraphNode node (ORB_SLAM2::System::RGBD, node_handle, image_transport);

    ros::spin();

    ros::shutdown();

    return 0;
}



PlanningGraphNode::PlanningGraphNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport) : 
Node (sensor, node_handle, image_transport)

{
  Init(node_handle);
  rgb_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/rgb/image_raw", 1);
  depth_subscriber_ = new message_filters::Subscriber<sensor_msgs::Image> (node_handle, "/camera/depth_registered/image_raw", 1);

jump=0;
  sync_ = new message_filters::Synchronizer<sync_pol> (sync_pol(10), *rgb_subscriber_, *depth_subscriber_);
  sync_->registerCallback(boost::bind(&PlanningGraphNode::ImageCallback, this, _1, _2));
}


PlanningGraphNode::~PlanningGraphNode () {
  delete rgb_subscriber_;
  delete depth_subscriber_;
  delete sync_;
}


void PlanningGraphNode::ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB, const sensor_msgs::ImageConstPtr& msgD) {
  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try {
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try {
    cv_ptrD = cv_bridge::toCvShare(msgD);
  } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
  }
 
  current_frame_time_ = msgRGB->header.stamp;

 orb_slam_->TrackRGBD(cv_ptrRGB->image,cv_ptrD->image,cv_ptrRGB->header.stamp.toSec());
  std_msgs::Int8 status_msg;
status_msg.data = orb_slam_->getTrackingState();
  if(status_msg.data==2&&jump>30)
  {
  posegraph = Build();
  jump=0;
  }
  status_pub.publish(status_msg);
jump++;
  Update ();
}


void PlanningGraphNode::Init(ros::NodeHandle &node_handle)
{
  posegraph_pub = node_handle.advertise<visualization_msgs::Marker> ("briq/posegraph", 1);
  status_pub = node_handle.advertise<std_msgs::Int8>("briq/status_orb", 10);
  PathService=node_handle.advertiseService("briq/get_path", &PlanningGraphNode::GetPathService, this);
}


NavGraph PlanningGraphNode::Build()
{
 
 
  NavGraph graph;

  // vector<vector<float>> PoseList=orb_slam_->GetPoseGraph();
  //  pcl::PointCloud<pcl::PointXYZ> posecloud;
  //   sensor_msgs::PointCloud2 poseoutput;
    std::map<int,KeyFrame*> KfNode;
    std::map<int,vector<KeyFrame*>> PoseGraph;
    orb_slam_->GetPoseGraph(KfNode,PoseGraph);
  
    for(auto it=PoseGraph.begin();it!=PoseGraph.end();++it)
    {
      vector<KeyFrame*> NeiboList=it->second;
      //std::cout<<"node="<<it->first<<"->"<<NeiboList.size()<<std::endl;
      int counter=0;
      pcl::PointXYZ vertex;
      geometry_msgs::Point p1;
      cv::Mat t1  =KfNode[it->first]->GetCameraCenter();
      p1.x=  t1.at<float>(2);
      p1.y= -t1.at<float>(0);
      p1.z= -t1.at<float>(1);
      graph.AddVertex(it->first,p1.x,p1.y,p1.z);

      for(auto n:NeiboList)
      {
        geometry_msgs::Point p2;

        cv::Mat t2  =n->GetCameraCenter();
        p2.x=  t2.at<float>(2);
        p2.y= -t2.at<float>(0);
        p2.z= -t2.at<float>(1);
        double tmp=(p1.x-p2.x)*(p1.x-p2.x)+(p1.y-p2.y)*(p1.y-p2.y)+(p1.z-p2.z)*(p1.z-p2.z);
        float dist=sqrt(tmp);
        if(dist<1.5)
        {
          graph.AddEdge(it->first,n->mnId);
          // graphviz.points.push_back(p1);
          // graphviz.points.push_back(p2);
        }
 
      }
    }

  return graph;
}

vector<point3d> PlanningGraphNode::Planning(NavGraph graph,vector<float> Goal)
{
     visualization_msgs::Marker graphviz;
     cout<<"planning!"<<endl;
    graphviz.header.frame_id =  "map";
    graphviz.header.stamp = ros::Time::now();
    graphviz.action= visualization_msgs::Marker::ADD;
    graphviz.pose.orientation.w = 1.0;
    graphviz.type = visualization_msgs::Marker::LINE_STRIP;
    graphviz.color.b = 1.0;
    graphviz.color.a = 1.0;
    graphviz.scale.x = 0.1;
  
    cv::Mat pos=orb_slam_->GetCurrentPosition();
    vector<float> rpos=TransformFromMat(pos,true);
    point3d init;
    init.x=rpos[0];
    init.y=rpos[1];
    init.z=rpos[2];
    point3d goal(Goal[0],Goal[1],Goal[2]);
    vector<point3d> path;
    graph.AstarSearch(goal,init,path);
    geometry_msgs::Point pp;
      
    for(auto pn:path)
    {
      pp.x=pn.x;
      pp.y=pn.y;
      pp.z=pn.z;
      graphviz.points.push_back(pp);
    }

    posegraph_pub.publish(graphviz);
    return path;
}

vector<point3d> PlanningGraphNode::GoHome(NavGraph graph)
{
    cout<<"go home!"<<endl;
     visualization_msgs::Marker graphviz;
    graphviz.header.frame_id =  "map";
    graphviz.header.stamp = ros::Time::now();
    graphviz.action= visualization_msgs::Marker::ADD;
    graphviz.pose.orientation.w = 1.0;
    graphviz.type = visualization_msgs::Marker::LINE_STRIP;
    graphviz.color.g = 1.0;
    graphviz.color.a = 1.0;
    graphviz.scale.x = 0.1;
  
    cv::Mat pos=orb_slam_->GetCurrentPosition();
    vector<float> rpos=TransformFromMat(pos,true);
    point3d init;
    init.x=rpos[0];
    init.y=rpos[1];
    init.z=rpos[2];
    point3d goal=graph.VertexList[0];
    vector<point3d> path;
    graph.AstarSearch(goal,init,path);
    geometry_msgs::Point pp;
      
    for(auto pn:path)
    {
      pp.x=pn.x;
      pp.y=pn.y;
      pp.z=pn.z;
      graphviz.points.push_back(pp);
    }
  
    posegraph_pub.publish(graphviz);
    return path;
}

bool PlanningGraphNode::GetPathService( orb_slam2_ros::GetPath::Request &req,  orb_slam2_ros::GetPath::Response &res)
{
  vector<point3d> path ;
  if(req.gohome==true)
  {
    path = GoHome(posegraph);
  }else
  {
    vector<float> goal={req.goal.x,req.goal.y,req.goal.z};
   path = Planning(posegraph,goal);
  }
  for(auto p:path)
  {
    geometry_msgs::Point pp;
    pp.x=p.x;
    pp.y=p.y;
    pp.z=p.z;
    res.path.push_back(pp);
  }
  return true;
}