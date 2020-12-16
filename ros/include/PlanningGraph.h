/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef ORBSLAM2_ROS_PlanningGraphODE_H_
#define ORBSLAM2_ROS_PlanningGraphODE_H_

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/Int8.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/GetPlan.h>

#include <opencv2/core/core.hpp>
#include <tf/transform_broadcaster.h>

#include "System.h"
#include "KeyFrame.h"
#include "Node.h"
#include "orb_slam2_ros/GetPath.h"
#include<queue>
using namespace std;
using namespace ORB_SLAM2;


// class Graph
// {
//   public:
//   Graph(std::map<int,KeyFrame*> nodes,std::map<int,int>> adjlist)
//   {
//     Vertex=nodes;
//     NeiborKF = adjlist;

//   }
//   vector<int> Astar_search(vector<float> Current,vector<float> Goal)
//   {
      
//   }

//     vector<int> Dijstra_search(vector<float> Current,vector<float> Goal)
//   {
      
//   }


//   private:
//     std::map<int,KeyFrame*> Vertex ;
//     std::map<int,int> NeiboKF;
//     priority_queue<int,vector<int>,cmp> Q;
// }
using point3d=pcl::PointXYZ ;
struct mark
{
  int mnId;
  float cost;
  point3d pos;
  int parentId;

};

struct cmp
{
    bool operator() (const mark &a,const mark &b)
    {
        return a.cost > b.cost;
    }
};
class NavGraph
{
  public:
      std::map<int,point3d> VertexList;
      std::map<int,vector<int>> AdjList;

      void AddVertex(int mnId,float x,float y,float z)
      {
        point3d vertex;
        vertex.x=x;
        vertex.y=y;
        vertex.z=z;
        VertexList[mnId]=vertex;
      }

      void AddEdge(int mnIdA,int mnIdB)
      {
        if(AdjList[mnIdA].empty()==true)
        {
          vector<int> tmp;
          tmp.push_back(mnIdB);
          AdjList[mnIdA]=tmp;
        }else
        {
          vector<int> tmp = AdjList[mnIdA];
          tmp.push_back(mnIdB);
          AdjList[mnIdA]=tmp;
        }
        
      }

      bool AstarSearch(point3d Goal,point3d Init,vector<point3d> &Path)
      {
        int start_mnId,end_mnId;
        priority_queue <mark,vector<mark>,cmp> openList;
        map<int,bool> openListCheck;
        map<int,mark> closeList;
        // vector<mark> closeListV;
        searchVertex(Goal,Init,start_mnId,end_mnId);//搜索初始节点
        if(start_mnId==end_mnId)//目标就在附近
        {
          
          Path.push_back(Goal);
          //Path.push_back(VertexList[start_mnId]);
          Path.push_back(Init);
          // cout<<"find!"<<endl;
          cout<<"shortpath!"<<endl;
          return true;
        }

        float cost = cost_function(VertexList[start_mnId],VertexList[start_mnId],VertexList[end_mnId]);
        mark st = vertex2mark(start_mnId,VertexList[start_mnId],0,start_mnId);

        openList.push(st);
        openListCheck[st.mnId]=true;
        mark n = openList.top();
        while(openList.empty()==false )//如果openset不为空
        {
          n = openList.top();//选取优先级最高的节点n
          closeList[n.mnId]=n;//加入到closeset
          
          // closeListV.push_back(n);//路径
          if(n.mnId==end_mnId)//如果节点n为终点
          {

            cout<<"find= "<<start_mnId<<" to "<<n.mnId<<endl;
            // for(auto cit=closeList.begin();cit!=closeList.end();cit++)
            // {
            //   cout<<"mnId="<<cit->first<<","<<cit->second.cost<<","<<cit->second.parentId<<endl;
            // }
            Path.push_back(Goal);
            while(n.parentId!=start_mnId)
            {
              cout<<"("<<n.mnId<<")"<<"->";
              Path.push_back(n.pos);
              n=closeList[n.parentId];
              
            }
            cout<<"("<<start_mnId<<endl;
            cout<<endl;
            Path.push_back(VertexList[start_mnId]);
            Path.push_back(Init);
            return true;
          }
                                      //如果不是终点
          openList.pop();//从openset删除节点n
          openListCheck[n.mnId]=false;
          
          vector<int> neiboIds = AdjList[n.mnId];//遍历节点n的所有近邻

          for(auto nId:neiboIds)
          {
            if(closeList.count(nId)==0 && (openListCheck.count(nId)==0||openListCheck[nId]==false))//如果节点m不在
            {
              //妹找过
              cost = cost_function(VertexList[nId],VertexList[start_mnId],VertexList[end_mnId]);
              mark m=vertex2mark(nId,VertexList[nId],cost,n.mnId);
              openList.push(m);
              openListCheck[m.mnId]=true;

            }
          }
          
        }
        return false;

      }

      

      mark vertex2mark(int mnId,point3d p,float cost,int parent)
      {
        mark n;
        n.cost=cost;
        n.mnId=mnId;
        n.pos=p;
        n.parentId=parent;
        return n;
      }

      float cost_function(point3d v,point3d s,point3d e)
      {
        float g = Distance(v,s);
        float h = Distance(v,e);
        return 0.8*g + 0.2*h;
      }

      void searchVertex(point3d Goal,point3d Init,int &start, int &end)
      {
        float GoaltmpDist;
        float InittmpDist;
        
        bool status=false;
        for(auto it=VertexList.begin();it!=VertexList.end();++it)
        {
          float Gdist=Distance(Goal,it->second);
          float Idist=Distance(Init,it->second);
          if(status==false)
          {
            status=true;
            GoaltmpDist=Gdist;
            InittmpDist=Idist;
            start=it->first;
            end=it->first;
          }
          if(Gdist<GoaltmpDist)
          {
            GoaltmpDist=Gdist;
            end=it->first;
          }

          if(Idist<InittmpDist)
          {
            InittmpDist=Idist;
            start=it->first;
          }

        }
      }

      float Distance(point3d A,point3d B)
      {
        return sqrt((A.x-B.x)*(A.x-B.x)+(A.y-B.y)*(A.y-B.y)+(A.z-B.z)*(A.z-B.z));
      }
  private:


};
class PlanningGraphNode : public Node
{
  public:
    PlanningGraphNode (const ORB_SLAM2::System::eSensor sensor, ros::NodeHandle &node_handle, image_transport::ImageTransport &image_transport);
    ~PlanningGraphNode ();
    void ImageCallback (const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
  void Init(ros::NodeHandle &node_handle);
  NavGraph Build();
  vector<point3d> Planning(NavGraph graph,vector<float> Goal);
  vector<point3d> GoHome(NavGraph graph);
bool GetPathService( orb_slam2_ros::GetPath::Request &req,orb_slam2_ros::GetPath::Response &res);
  private:
    cv::Mat nowPose;
    ros::Publisher posegraph_pub;
    ros::Publisher status_pub;
    int jump;
    NavGraph posegraph;
    ros::ServiceServer PathService;
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> sync_pol;
    message_filters::Subscriber<sensor_msgs::Image> *rgb_subscriber_;
    message_filters::Subscriber<sensor_msgs::Image> *depth_subscriber_;
    message_filters::Synchronizer<sync_pol> *sync_;
};

#endif //ORBSLAM2_ROS_PlanningGraphODE_H_
