// Copyright 2013, Ji Zhang, Carnegie Mellon University
// Further contributions copyright (c) 2016, Southwest Research Institute
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 3. Neither the name of the copyright holder nor the names of its
//    contributors may be used to endorse or promote products derived from this
//    software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// This is an implementation of the algorithm described in the following paper:
//   J. Zhang and S. Singh. LOAM: Lidar Odometry and Mapping in Real-time.
//     Robotics: Science and Systems Conference (RSS). Berkeley, CA, July 2014.
//   T. Shan and B. Englot. LeGO-LOAM: Lightweight and Ground-Optimized Lidar Odometry and Mapping on Variable Terrain
//      IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS). October 2018.

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>

#include <unistd.h>

#include <gtsam/nonlinear/ISAM2.h>

#include "Scancontext.h"
#include "utility.h"

#include <pcl/io/pcd_io.h>
#include <condition_variable>
#include <unistd.h>

#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
//#include "gnss_bestpose.h"
#include "GPS2UTM.cpp"
#include <pcl/filters/passthrough.h>

std::string findIndex(string ori, /*string b, */string c)
{
    int index_f, index_l;
    int index_gap;
    // index_f = a.find(b) + 4;
    index_f = 0;
    index_l = ori.find(c);
    index_gap = index_l - index_f;
    return (ori.substr(index_f, index_gap)).c_str();
}
int cmp(string a, string b)
{
    double c, d;
    // c = findIndex(a, "Coin", ".bin");
    // d = findIndex(b, "Coin", ".bin");
    c = atof(findIndex(findIndex(a, "th_keyframe"), "_").c_str());
    d = atof(findIndex(findIndex(b, "th_keyframe"), "_").c_str());
    return  c < d;
}
vector<string> getFiles(string cate_dir)
{
    vector<string> files; //存放文件名
    DIR *dir;
    struct dirent *ptr;
    char base[1000];

    if ((dir = opendir(cate_dir.c_str())) == NULL)
    {
        perror("Open dir error...");
        exit(1);
    }

    while ((ptr = readdir(dir)) != NULL)
    {
        if (strcmp(ptr->d_name, ".") == 0 || strcmp(ptr->d_name, "..") == 0) ///current dir OR parrent dir
            continue;
        else if (ptr->d_type == 8) ///file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            files.push_back(ptr->d_name);
        else if (ptr->d_type == 10) ///link file
            //printf("d_name:%s/%s\n",basePath,ptr->d_name);
            continue;
        else if (ptr->d_type == 4) ///dir
        {
            files.push_back(ptr->d_name);
            /*
		        memset(base,'\0',sizeof(base));
		        strcpy(base,basePath);
		        strcat(base,"/");
		        strcat(base,ptr->d_nSame);
		        readFileList(base);
			*/
        }
    }
    closedir(dir);

    //排序，按从小到大排序
    sort(files.begin(), files.end(), cmp);
    return files;
}

void LonLat2UTM(double longitude, double latitude, double &UTME, double &UTMN)
{
    UTM_structure result = LatLonToUTMXY(longitude, latitude);
    UTME = result.x;
    UTMN = result.y;
    if (result.zone != 51)
    {
        perror("wrong utm zone!");
        exit(1);
    }
}

using namespace gtsam;

class mapOptimization
{

private:
    std::mutex mtx_cv;
    std::condition_variable cv;
    bool premap_processed;
    bool sci_localized;
    int localizeResultIndex;
    float YawDiff;
    vector<string> files;
    pcl::PointCloud<PointType>::Ptr keyframeMap;
    int findGT_plus, findPOS_plus;

    NonlinearFactorGraph gtSAMgraph;
    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;

    noiseModel::Diagonal::shared_ptr priorNoise;
    noiseModel::Diagonal::shared_ptr odometryNoise;
    noiseModel::Diagonal::shared_ptr constraintNoise;
    noiseModel::Base::shared_ptr robustNoiseModel;

    ros::NodeHandle nh;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubOdomAftMapped;
    ros::Publisher pubKeyPoses;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRegisteredCloud;
    ros::Publisher pubRelocalTarget, pubRelocalSource, pubRelocalICP;
    ros::Publisher pubTransformCloud;
    ros::Publisher pubKeyframeMap;

    ros::Subscriber subLaserCloudRaw;
    ros::Subscriber subLaserCloudCornerLast;
    ros::Subscriber subLaserCloudSurfLast;
    ros::Subscriber subOutlierCloudLast;
    ros::Subscriber subLaserOdometry;
    ros::Subscriber subImu;
    ros::Subscriber subGPS;

    nav_msgs::Odometry odomAftMapped;
    tf::StampedTransform aftMappedTrans;
    tf::TransformBroadcaster tfBroadcaster;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> outlierCloudKeyFrames;

    deque<pcl::PointCloud<PointType>::Ptr> recentCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> recentOutlierCloudKeyFrames;
    int latestFrameID;

    vector<int> surroundingExistingKeyPosesID;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingCornerCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingSurfCloudKeyFrames;
    deque<pcl::PointCloud<PointType>::Ptr> surroundingOutlierCloudKeyFrames;

    PointType previousRobotPosPoint;
    PointType currentRobotPosPoint;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr surroundingKeyPoses;
    pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS;

    pcl::PointCloud<PointType>::Ptr laserCloudRaw;
    double laserCloudRawTime;
    pcl::PointCloud<PointType>::Ptr laserCloudRawDS;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLast;   // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudOutlierLastDS; // corner feature set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLast;   // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfTotalLastDS; // downsampled corner featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::PointCloud<PointType>::Ptr RSlatestSurfKeyFrameCloud; // giseop, RS: radius search
    pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr RSnearHistorySurfKeyFrameCloudDS;

    pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr nearHistoryCornerKeyFrameCloudDS;
    pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr SCnearHistorySurfKeyFrameCloudDS;

    pcl::PointCloud<PointType>::Ptr latestCornerKeyFrameCloud;
    pcl::PointCloud<PointType>::Ptr SClatestSurfKeyFrameCloud; // giseop, SC: scan context
    pcl::PointCloud<PointType>::Ptr latestSurfKeyFrameCloudDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPoses;
    pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFrames;
    pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS;

    std::vector<int> pointSearchInd;
    std::vector<float> pointSearchSqDis;

    pcl::VoxelGrid<PointType> downSizeFilterScancontext;
    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterOutlier;
    pcl::VoxelGrid<PointType> downSizeFilterHistoryKeyFrames;    // for histor key frames of loop closure
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;   // for global map visualization
    pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;  // for global map visualization

    double timeLaserCloudCornerLast;
    double timeLaserCloudSurfLast;
    double timeLaserOdometry;
    double timeLaserCloudOutlierLast;
    double timeLastGloalMapPublish;

    bool newLaserCloudCornerLast;
    bool newLaserCloudSurfLast;
    bool newLaserOdometry;
    bool newLaserCloudOutlierLast;

    float transformLast[6];
    float transformSum[6];
    float transformIncre[6];
    float transformTobeMapped[6];
    float transformBefMapped[6];
    float transformAftMapped[6];

    int imuPointerFront;
    int imuPointerLast;

    double imuTime[imuQueLength];
    float imuRoll[imuQueLength];
    float imuPitch[imuQueLength];

    std::mutex mtx;

    double timeLastProcessing;

    PointType pointOri, pointSel, pointProj, coeff;

    cv::Mat matA0;
    cv::Mat matB0;
    cv::Mat matX0;

    cv::Mat matA1;
    cv::Mat matD1;
    cv::Mat matV1;

    bool isDegenerate;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum;
    int laserCloudSurfFromMapDSNum;
    int laserCloudCornerLastDSNum;
    int laserCloudSurfLastDSNum;
    int laserCloudOutlierLastDSNum;
    int laserCloudSurfTotalLastDSNum;

    bool potentialLoopFlag;
    double timeSaveFirstCurrentScanForLoopClosure;
    int RSclosestHistoryFrameID;
    int SCclosestHistoryFrameID; // giseop
    int latestFrameIDLoopCloure;
    float yawDiffRad;

    bool aLoopIsClosed;

    float cRoll, sRoll, cPitch, sPitch, cYaw, sYaw, tX, tY, tZ;
    float ctRoll, stRoll, ctPitch, stPitch, ctYaw, stYaw, tInX, tInY, tInZ;

public:
    bool LocalizeFlag, generateMap;
    string Scene;
    int MapFrameNum;
    std::string SceneFolder;
    int relocalHz, relocalNum;
    std::string evalPath, mapPospath, gtpath;
    std::ofstream evaluationTxt;
    std::vector<geometry_msgs::PoseStamped> gt_load;
    std::vector<geometry_msgs::PoseStamped> mappos_load;
    double distThreshold;
    int save_startind;
    bool saveRelocalKeyFrame;
    bool pcdStillNotSave;
    int skipFusionHz, skipFusionNum;
    int relocalSaveInd;

    // // loop detector
    SCManager scManager, forTransform;

public:
    mapOptimization() : nh("~")
    {
        LocalizeFlag = false;
        generateMap = false;
        saveRelocalKeyFrame = false;
        pcdStillNotSave = false;
        skipFusionHz = 10; // 10Hz
        skipFusionNum = 10;
        relocalSaveInd = 0;

        premap_processed = false;
        sci_localized = false;
        relocalNum = 10;
        findGT_plus = 0;
        findPOS_plus = 0;

        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.01;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("/key_pose_origin", 2);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("/laser_cloud_surround", 2);
        pubOdomAftMapped = nh.advertise<nav_msgs::Odometry>("/aft_mapped_to_init", 5);

        subLaserCloudRaw = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 1000, &mapOptimization::laserCloudRawHandler, this);
        subLaserCloudCornerLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_corner_last", 2, &mapOptimization::laserCloudCornerLastHandler, this);
        subLaserCloudSurfLast = nh.subscribe<sensor_msgs::PointCloud2>("/laser_cloud_surf_last", 2, &mapOptimization::laserCloudSurfLastHandler, this);
        subOutlierCloudLast = nh.subscribe<sensor_msgs::PointCloud2>("/outlier_cloud_last", 2, &mapOptimization::laserCloudOutlierLastHandler, this);
        subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/laser_odom_to_init", 5, &mapOptimization::laserOdometryHandler, this);
        subImu = nh.subscribe<sensor_msgs::Imu>(imuTopic, 50, &mapOptimization::imuHandler, this);
        subGPS = nh.subscribe("/fusion", 10000, &mapOptimization::FusionCallback, this);

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/history_cloud", 2);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/corrected_cloud", 2);
        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("/recent_cloud", 2);
        pubRegisteredCloud = nh.advertise<sensor_msgs::PointCloud2>("/registered_cloud", 2);
        pubRelocalTarget = nh.advertise<sensor_msgs::PointCloud2>("/relocal_target", 2);
        pubRelocalSource = nh.advertise<sensor_msgs::PointCloud2>("/relocal_source", 2);
        pubRelocalICP = nh.advertise<sensor_msgs::PointCloud2>("/relocal_icp", 2);
        pubTransformCloud = nh.advertise<sensor_msgs::PointCloud2>("/transform", 2);
        pubKeyframeMap = nh.advertise<sensor_msgs::PointCloud2>("/keyframe_map", 10);

        float filter_size;
        downSizeFilterCorner.setLeafSize(0.2, 0.2, 0.2);
        filter_size = 0.5;
        downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
        filter_size = 0.3;
        downSizeFilterSurf.setLeafSize(filter_size, filter_size, filter_size); // default 0.4;
        downSizeFilterOutlier.setLeafSize(0.4, 0.4, 0.4);

        filter_size = 0.3;
        downSizeFilterHistoryKeyFrames.setLeafSize(filter_size, filter_size, filter_size); // default 0.4; for histor key frames of loop closure
        filter_size = 1.0;
        downSizeFilterSurroundingKeyPoses.setLeafSize(filter_size, filter_size, filter_size); // default 1; for surrounding key poses of scan-to-map optimization

        downSizeFilterGlobalMapKeyPoses.setLeafSize(1.0, 1.0, 1.0);  // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(0.4, 0.4, 0.4); // for global map visualization

        odomAftMapped.header.frame_id = "/camera_init";
        odomAftMapped.child_frame_id = "/aft_mapped";

        aftMappedTrans.frame_id_ = "/camera_init";
        aftMappedTrans.child_frame_id_ = "/aft_mapped";

        allocateMemory();
    }

    /*
    不用GPS间隔建图时生成GPS信息
    void FusionCallback(const geometry_msgs::PoseStamped& gps_msg)
    {
        if(saveRelocalKeyFrame && !pcdStillNotSave)
        {
            double latitude = gps_msg.pose.position.x;
            double longitude = gps_msg.pose.position.y;
            LonLat2UTM(longitude,latitude,scManager.utm_EN.first,scManager.utm_EN.second); 
            std::ofstream gpsOut;
            gpsOut.open(gpstxt_path, std::ios::app);
            gpsOut<<setprecision(15)<<scManager.utm_EN.first<<" "<<scManager.utm_EN.second<<endl;
            pcdStillNotSave = true;
        }
    }
*/

    // void getRelocalParam(bool initLocalizeFlag, bool initgenerateMap, string initScene,
    //                      int initMapFrameNum, double Threshold, int Hz, std::string gpsPath,
    //                      std::string gpsFailPath, double gpsDistThreshold,
    //                      std::string descriptor)
    // {
    //     LocalizeFlag = initLocalizeFlag;
    //     generateMap = initgenerateMap;
    //     Scene = initScene;
    //     MapFrameNum = initMapFrameNum;
    //     SceneFolder = getenv("HOME");
    //     SceneFolder += "/catkin_ws/data/pre_map/" + Scene + "/";
    //     scManager.setThres(Threshold);
    //     scManager.setgpsFailPath(gpsFailPath);
    //     scManager.setDescriptor(descriptor);
    //     relocalHz = Hz;
    //     gpstxt_path = gpsPath;
    //     distThreshold = gpsDistThreshold;
    //     cout << "LocalizeFlag: " << LocalizeFlag << endl;
    //     cout << "Scene: " << Scene << endl;
    //     cout << "MapFrameNum: " << MapFrameNum << endl;
    //     cout << "SceneFolder: " << SceneFolder << endl;
    //     cout << "Threshold: " << Threshold << endl;
    //     cout << "relocalHz: " << relocalHz << endl;
    //     cout << "distThreshold: " << distThreshold << endl;
    //     cout << "gpstxt_path: " << gpstxt_path << endl;
    //     cout << "gpsFailPath: " << gpsFailPath << endl;
    //     cout << "descriptor: " << descriptor << endl;
    // }

    void allocateMemory()
    {
        keyframeMap.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        surroundingKeyPoses.reset(new pcl::PointCloud<PointType>());
        surroundingKeyPosesDS.reset(new pcl::PointCloud<PointType>());

        laserCloudRaw.reset(new pcl::PointCloud<PointType>());             // corner feature set from odoOptimization
        laserCloudRawDS.reset(new pcl::PointCloud<PointType>());           // corner feature set from odoOptimization
        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());      // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());        // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>());    // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());      // downsampled surf featuer set from odoOptimization
        laserCloudOutlierLast.reset(new pcl::PointCloud<PointType>());     // corner feature set from odoOptimization
        laserCloudOutlierLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled corner feature set from odoOptimization
        laserCloudSurfTotalLast.reset(new pcl::PointCloud<PointType>());   // surf feature set from odoOptimization
        laserCloudSurfTotalLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        nearHistoryCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        nearHistoryCornerKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());
        SCnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        SCnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        latestCornerKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        SClatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        latestSurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        RSlatestSurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>()); // giseop
        RSnearHistorySurfKeyFrameCloud.reset(new pcl::PointCloud<PointType>());
        RSnearHistorySurfKeyFrameCloudDS.reset(new pcl::PointCloud<PointType>());

        kdtreeGlobalMap.reset(new pcl::KdTreeFLANN<PointType>());
        globalMapKeyPoses.reset(new pcl::PointCloud<PointType>());
        globalMapKeyPosesDS.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFrames.reset(new pcl::PointCloud<PointType>());
        globalMapKeyFramesDS.reset(new pcl::PointCloud<PointType>());

        timeLaserCloudCornerLast = 0;
        timeLaserCloudSurfLast = 0;
        timeLaserOdometry = 0;
        timeLaserCloudOutlierLast = 0;
        timeLastGloalMapPublish = 0;

        timeLastProcessing = -1;

        newLaserCloudCornerLast = false;
        newLaserCloudSurfLast = false;

        newLaserOdometry = false;
        newLaserCloudOutlierLast = false;

        for (int i = 0; i < 6; ++i)
        {
            transformLast[i] = 0;
            transformSum[i] = 0;
            transformIncre[i] = 0;
            transformTobeMapped[i] = 0;
            transformBefMapped[i] = 0;
            transformAftMapped[i] = 0;
        }

        imuPointerFront = 0;
        imuPointerLast = -1;

        for (int i = 0; i < imuQueLength; ++i)
        {
            imuTime[i] = 0;
            imuRoll[i] = 0;
            imuPitch[i] = 0;
        }

        gtsam::Vector Vector6(6);
        Vector6 << 1e-6, 1e-6, 1e-6, 1e-8, 1e-8, 1e-6;
        priorNoise = noiseModel::Diagonal::Variances(Vector6);
        odometryNoise = noiseModel::Diagonal::Variances(Vector6);

        matA0 = cv::Mat(5, 3, CV_32F, cv::Scalar::all(0));
        matB0 = cv::Mat(5, 1, CV_32F, cv::Scalar::all(-1));
        matX0 = cv::Mat(3, 1, CV_32F, cv::Scalar::all(0));

        matA1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));
        matD1 = cv::Mat(1, 3, CV_32F, cv::Scalar::all(0));
        matV1 = cv::Mat(3, 3, CV_32F, cv::Scalar::all(0));

        isDegenerate = false;
        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));

        laserCloudCornerFromMapDSNum = 0;
        laserCloudSurfFromMapDSNum = 0;
        laserCloudCornerLastDSNum = 0;
        laserCloudSurfLastDSNum = 0;
        laserCloudOutlierLastDSNum = 0;
        laserCloudSurfTotalLastDSNum = 0;

        potentialLoopFlag = false;
        aLoopIsClosed = false;

        latestFrameID = 0;
    }

    void transformAssociateToMap()
    {
        float x1 = cos(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) - sin(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);
        float y1 = transformBefMapped[4] - transformSum[4];
        float z1 = sin(transformSum[1]) * (transformBefMapped[3] - transformSum[3]) + cos(transformSum[1]) * (transformBefMapped[5] - transformSum[5]);

        float x2 = x1;
        float y2 = cos(transformSum[0]) * y1 + sin(transformSum[0]) * z1;
        float z2 = -sin(transformSum[0]) * y1 + cos(transformSum[0]) * z1;

        transformIncre[3] = cos(transformSum[2]) * x2 + sin(transformSum[2]) * y2;
        transformIncre[4] = -sin(transformSum[2]) * x2 + cos(transformSum[2]) * y2;
        transformIncre[5] = z2;

        float sbcx = sin(transformSum[0]);
        float cbcx = cos(transformSum[0]);
        float sbcy = sin(transformSum[1]);
        float cbcy = cos(transformSum[1]);
        float sbcz = sin(transformSum[2]);
        float cbcz = cos(transformSum[2]);

        float sblx = sin(transformBefMapped[0]);
        float cblx = cos(transformBefMapped[0]);
        float sbly = sin(transformBefMapped[1]);
        float cbly = cos(transformBefMapped[1]);
        float sblz = sin(transformBefMapped[2]);
        float cblz = cos(transformBefMapped[2]);

        float salx = sin(transformAftMapped[0]);
        float calx = cos(transformAftMapped[0]);
        float saly = sin(transformAftMapped[1]);
        float caly = cos(transformAftMapped[1]);
        float salz = sin(transformAftMapped[2]);
        float calz = cos(transformAftMapped[2]);

        float srx = -sbcx * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz) - cbcx * sbcy * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - cbcx * cbcy * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx);
        transformTobeMapped[0] = -asin(srx);

        float srycrx = sbcx * (cblx * cblz * (caly * salz - calz * salx * saly) - cblx * sblz * (caly * calz + salx * saly * salz) + calx * saly * sblx) - cbcx * cbcy * ((caly * calz + salx * saly * salz) * (cblz * sbly - cbly * sblx * sblz) + (caly * salz - calz * salx * saly) * (sbly * sblz + cbly * cblz * sblx) - calx * cblx * cbly * saly) + cbcx * sbcy * ((caly * calz + salx * saly * salz) * (cbly * cblz + sblx * sbly * sblz) + (caly * salz - calz * salx * saly) * (cbly * sblz - cblz * sblx * sbly) + calx * cblx * saly * sbly);
        float crycrx = sbcx * (cblx * sblz * (calz * saly - caly * salx * salz) - cblx * cblz * (saly * salz + caly * calz * salx) + calx * caly * sblx) + cbcx * cbcy * ((saly * salz + caly * calz * salx) * (sbly * sblz + cbly * cblz * sblx) + (calz * saly - caly * salx * salz) * (cblz * sbly - cbly * sblx * sblz) + calx * caly * cblx * cbly) - cbcx * sbcy * ((saly * salz + caly * calz * salx) * (cbly * sblz - cblz * sblx * sbly) + (calz * saly - caly * salx * salz) * (cbly * cblz + sblx * sbly * sblz) - calx * caly * cblx * sbly);
        transformTobeMapped[1] = atan2(srycrx / cos(transformTobeMapped[0]),
                                       crycrx / cos(transformTobeMapped[0]));

        float srzcrx = (cbcz * sbcy - cbcy * sbcx * sbcz) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) - (cbcy * cbcz + sbcx * sbcy * sbcz) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) + cbcx * sbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        float crzcrx = (cbcy * sbcz - cbcz * sbcx * sbcy) * (calx * calz * (cbly * sblz - cblz * sblx * sbly) - calx * salz * (cbly * cblz + sblx * sbly * sblz) + cblx * salx * sbly) - (sbcy * sbcz + cbcy * cbcz * sbcx) * (calx * salz * (cblz * sbly - cbly * sblx * sblz) - calx * calz * (sbly * sblz + cbly * cblz * sblx) + cblx * cbly * salx) + cbcx * cbcz * (salx * sblx + calx * cblx * salz * sblz + calx * calz * cblx * cblz);
        transformTobeMapped[2] = atan2(srzcrx / cos(transformTobeMapped[0]),
                                       crzcrx / cos(transformTobeMapped[0]));

        x1 = cos(transformTobeMapped[2]) * transformIncre[3] - sin(transformTobeMapped[2]) * transformIncre[4];
        y1 = sin(transformTobeMapped[2]) * transformIncre[3] + cos(transformTobeMapped[2]) * transformIncre[4];
        z1 = transformIncre[5];

        x2 = x1;
        y2 = cos(transformTobeMapped[0]) * y1 - sin(transformTobeMapped[0]) * z1;
        z2 = sin(transformTobeMapped[0]) * y1 + cos(transformTobeMapped[0]) * z1;

        transformTobeMapped[3] = transformAftMapped[3] - (cos(transformTobeMapped[1]) * x2 + sin(transformTobeMapped[1]) * z2);
        transformTobeMapped[4] = transformAftMapped[4] - y2;
        transformTobeMapped[5] = transformAftMapped[5] - (-sin(transformTobeMapped[1]) * x2 + cos(transformTobeMapped[1]) * z2);
    }

    void transformUpdate()
    {
        if (imuPointerLast >= 0)
        {
            float imuRollLast = 0, imuPitchLast = 0;
            while (imuPointerFront != imuPointerLast)
            {
                if (timeLaserOdometry + scanPeriod < imuTime[imuPointerFront])
                {
                    break;
                }
                imuPointerFront = (imuPointerFront + 1) % imuQueLength;
            }

            if (timeLaserOdometry + scanPeriod > imuTime[imuPointerFront])
            {
                imuRollLast = imuRoll[imuPointerFront];
                imuPitchLast = imuPitch[imuPointerFront];
            }
            else
            {
                int imuPointerBack = (imuPointerFront + imuQueLength - 1) % imuQueLength;
                float ratioFront = (timeLaserOdometry + scanPeriod - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
                float ratioBack = (imuTime[imuPointerFront] - timeLaserOdometry - scanPeriod) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);

                imuRollLast = imuRoll[imuPointerFront] * ratioFront + imuRoll[imuPointerBack] * ratioBack;
                imuPitchLast = imuPitch[imuPointerFront] * ratioFront + imuPitch[imuPointerBack] * ratioBack;
            }

            transformTobeMapped[0] = 0.998 * transformTobeMapped[0] + 0.002 * imuPitchLast;
            transformTobeMapped[2] = 0.998 * transformTobeMapped[2] + 0.002 * imuRollLast;
        }

        for (int i = 0; i < 6; i++)
        {
            transformBefMapped[i] = transformSum[i];
            transformAftMapped[i] = transformTobeMapped[i];
        }
    }

    void updatePointAssociateToMapSinCos()
    {
        cRoll = cos(transformTobeMapped[0]);
        sRoll = sin(transformTobeMapped[0]);

        cPitch = cos(transformTobeMapped[1]);
        sPitch = sin(transformTobeMapped[1]);

        cYaw = cos(transformTobeMapped[2]);
        sYaw = sin(transformTobeMapped[2]);

        tX = transformTobeMapped[3];
        tY = transformTobeMapped[4];
        tZ = transformTobeMapped[5];
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        float x1 = cYaw * pi->x - sYaw * pi->y;
        float y1 = sYaw * pi->x + cYaw * pi->y;
        float z1 = pi->z;

        float x2 = x1;
        float y2 = cRoll * y1 - sRoll * z1;
        float z2 = sRoll * y1 + cRoll * z1;

        po->x = cPitch * x2 + sPitch * z2 + tX;
        po->y = y2 + tY;
        po->z = -sPitch * x2 + cPitch * z2 + tZ;
        po->intensity = pi->intensity;
    }

    void updateTransformPointCloudSinCos(PointTypePose *tIn)
    {

        ctRoll = cos(tIn->roll);
        stRoll = sin(tIn->roll);

        ctPitch = cos(tIn->pitch);
        stPitch = sin(tIn->pitch);

        ctYaw = cos(tIn->yaw);
        stYaw = sin(tIn->yaw);

        tInX = tIn->x;
        tInY = tIn->y;
        tInZ = tIn->z;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn)
    {
        // !!! DO NOT use pcl for point cloud transformation, results are not accurate
        // Reason: unkown
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i)
        {

            pointFrom = &cloudIn->points[i];
            float x1 = ctYaw * pointFrom->x - stYaw * pointFrom->y;
            float y1 = stYaw * pointFrom->x + ctYaw * pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = ctRoll * y1 - stRoll * z1;
            float z2 = stRoll * y1 + ctRoll * z1;

            pointTo.x = ctPitch * x2 + stPitch * z2 + tInX;
            pointTo.y = y2 + tInY;
            pointTo.z = -stPitch * x2 + ctPitch * z2 + tInZ;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }
        return cloudOut;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {

        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        PointType *pointFrom;
        PointType pointTo;

        int cloudSize = cloudIn->points.size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i)
        {

            pointFrom = &cloudIn->points[i];
            float x1 = cos(transformIn->yaw) * pointFrom->x - sin(transformIn->yaw) * pointFrom->y;
            float y1 = sin(transformIn->yaw) * pointFrom->x + cos(transformIn->yaw) * pointFrom->y;
            float z1 = pointFrom->z;

            float x2 = x1;
            float y2 = cos(transformIn->roll) * y1 - sin(transformIn->roll) * z1;
            float z2 = sin(transformIn->roll) * y1 + cos(transformIn->roll) * z1;

            pointTo.x = cos(transformIn->pitch) * x2 + sin(transformIn->pitch) * z2 + transformIn->x;
            pointTo.y = y2 + transformIn->y;
            pointTo.z = -sin(transformIn->pitch) * x2 + cos(transformIn->pitch) * z2 + transformIn->z;
            pointTo.intensity = pointFrom->intensity;

            cloudOut->points[i] = pointTo;
        }

        return cloudOut;
    }

    void laserCloudOutlierLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudOutlierLast = msg->header.stamp.toSec();
        laserCloudOutlierLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudOutlierLast);
        newLaserCloudOutlierLast = true;
    }

    void quat2euler(geometry_msgs::Quaternion quat, double &roll, double &pitch, double &yaw)
    {
        double w = quat.w;
        double x = quat.x;
        double y = quat.y;
        double z = quat.z;
        // cout << "[quat2euler] w: " << w << ", x: " << x << ", y: " << y << ", z: " << z << endl;
        double q11 = w*w;
        double q12 = w*x;
        double q13 = w*y;
        double q14 = w*z; 
        double q22 = x*x;
        double q23 = x*y;
        double q24 = x*z;     
        double q33 = y*y;
        double q34 = y*z;  
        double q44 = z*z;
        double C12=2*(q23-q14);
        double C22=q11-q22+q33-q44;
        double C31=2*(q24-q13);
        double C32=2*(q34+q12);
        double C33=q11-q22-q33+q44;
        pitch = asin(C32);
        roll = -atan2(C31, C33);
        yaw = -atan2(C12, C22);
    }

    void laserCloudCornerLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudCornerLast = msg->header.stamp.toSec();
        laserCloudCornerLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudCornerLast);
        newLaserCloudCornerLast = true;
    }

    void laserCloudSurfLastHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        timeLaserCloudSurfLast = msg->header.stamp.toSec();
        laserCloudSurfLast->clear();
        pcl::fromROSMsg(*msg, *laserCloudSurfLast);
        newLaserCloudSurfLast = true;
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &laserOdometry)
    {
        timeLaserOdometry = laserOdometry->header.stamp.toSec();
        double roll, pitch, yaw;
        geometry_msgs::Quaternion geoQuat = laserOdometry->pose.pose.orientation;
        tf::Matrix3x3(tf::Quaternion(geoQuat.z, -geoQuat.x, -geoQuat.y, geoQuat.w)).getRPY(roll, pitch, yaw);
        transformSum[0] = -pitch;
        transformSum[1] = -yaw;
        transformSum[2] = roll;
        transformSum[3] = laserOdometry->pose.pose.position.x;
        transformSum[4] = laserOdometry->pose.pose.position.y;
        transformSum[5] = laserOdometry->pose.pose.position.z;
        newLaserOdometry = true;
    }

    void imuHandler(const sensor_msgs::Imu::ConstPtr &imuIn)
    {
        double roll, pitch, yaw;
        tf::Quaternion orientation;
        tf::quaternionMsgToTF(imuIn->orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        imuPointerLast = (imuPointerLast + 1) % imuQueLength;
        imuTime[imuPointerLast] = imuIn->header.stamp.toSec();
        imuRoll[imuPointerLast] = roll;
        imuPitch[imuPointerLast] = pitch;
    }

    void publishTF()
    {

        geometry_msgs::Quaternion geoQuat = tf::createQuaternionMsgFromRollPitchYaw(transformAftMapped[2], -transformAftMapped[0], -transformAftMapped[1]);

        odomAftMapped.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        odomAftMapped.pose.pose.orientation.x = -geoQuat.y;
        odomAftMapped.pose.pose.orientation.y = -geoQuat.z;
        odomAftMapped.pose.pose.orientation.z = geoQuat.x;
        odomAftMapped.pose.pose.orientation.w = geoQuat.w;
        odomAftMapped.pose.pose.position.x = transformAftMapped[3];
        odomAftMapped.pose.pose.position.y = transformAftMapped[4];
        odomAftMapped.pose.pose.position.z = transformAftMapped[5];
        odomAftMapped.twist.twist.angular.x = transformBefMapped[0];
        odomAftMapped.twist.twist.angular.y = transformBefMapped[1];
        odomAftMapped.twist.twist.angular.z = transformBefMapped[2];
        odomAftMapped.twist.twist.linear.x = transformBefMapped[3];
        odomAftMapped.twist.twist.linear.y = transformBefMapped[4];
        odomAftMapped.twist.twist.linear.z = transformBefMapped[5];
        pubOdomAftMapped.publish(odomAftMapped);

        aftMappedTrans.stamp_ = ros::Time().fromSec(timeLaserOdometry);
        aftMappedTrans.setRotation(tf::Quaternion(-geoQuat.y, -geoQuat.z, geoQuat.x, geoQuat.w));
        aftMappedTrans.setOrigin(tf::Vector3(transformAftMapped[3], transformAftMapped[4], transformAftMapped[5]));
        tfBroadcaster.sendTransform(aftMappedTrans);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }

    void publishKeyPosesAndFrames()
    {

        if (pubKeyPoses.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*cloudKeyPoses3D, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubKeyPoses.publish(cloudMsgTemp);
        }

        if (pubRecentKeyFrames.getNumSubscribers() != 0)
        {
            sensor_msgs::PointCloud2 cloudMsgTemp;
            pcl::toROSMsg(*laserCloudSurfFromMapDS, cloudMsgTemp);
            cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
            cloudMsgTemp.header.frame_id = "/camera_init";
            pubRecentKeyFrames.publish(cloudMsgTemp);
        }

        if (pubRegisteredCloud.getNumSubscribers() != 0)
        {

            //            for (int j = 0; j <= 20;++j)
            {
                pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
                PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
                *cloudOut += *transformPointCloud(laserCloudCornerLastDS, &thisPose6D);
                *cloudOut += *transformPointCloud(laserCloudSurfTotalLast, &thisPose6D);
                int cloudSize = cloudOut->points.size();

                sensor_msgs::PointCloud2 cloudMsgTemp;
                pcl::toROSMsg(*cloudOut, cloudMsgTemp);
                cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
                cloudMsgTemp.header.frame_id = "/camera_init";
                pubRegisteredCloud.publish(cloudMsgTemp);
            }
        }
    }

    void FusionCallback(const geometry_msgs::PoseStamped &gps_msg)
    {
        skipFusionNum = skipFusionNum + skipFusionHz;
        if (generateMap)
        {
            if (pcdStillNotSave)
                return;
            if (skipFusionNum >= 10)
            {
                skipFusionNum = 0;
                double time = gps_msg.header.stamp.toSec();
                double latitude = gps_msg.pose.position.x;
                double longitude = gps_msg.pose.position.y;
                double height = gps_msg.pose.position.z;
                double qx = gps_msg.pose.orientation.x;
                double qy = gps_msg.pose.orientation.y;
                double qz = gps_msg.pose.orientation.z;
                double qw = gps_msg.pose.orientation.w;
                LonLat2UTM(longitude, latitude, scManager.utm_EN.first, scManager.utm_EN.second);
                // saveRelocalKeyFrame = true; //使用另一种采集策略记得取消注释
                if (scManager.utmRecord.empty())
                {
                    saveRelocalKeyFrame = true;
                    // cout << "saveRelocalKeyFrame = true | is empty" << endl;
                    scManager.utmRecord.push_back(scManager.utm_EN);
                    pcdStillNotSave = true;
                }
                else
                {
                    auto it = scManager.utmRecord.end() - 1;
                    if (sqrt((scManager.utm_EN.first - it->first) * (scManager.utm_EN.first - it->first) + (scManager.utm_EN.second - it->second) * (scManager.utm_EN.second - it->second)) >= distThreshold)
                    {
                        saveRelocalKeyFrame = true;
                        // cout << "saveRelocalKeyFrame = true | not empty" << endl;
                        // save position information
                        scManager.utmRecord.push_back(scManager.utm_EN);
                        pcdStillNotSave = true;
                    }
                }
            }
        }
    }

    void RelocalThread()
    {
        loadpremap();
    }

    void combineKeyframeMap()
    {
        keyframeMap->clear();
        double initRoll, initPitch, initHeading;
        int initIndex;
        for (int i = 0; i < MapFrameNum; i++)
        {
            string pcdpath = SceneFolder + files[i];
            pcl::PointCloud<PointType>::Ptr loadRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::io::loadPCDFile(pcdpath, *loadRawCloudKeyFrame);
            std::string curMapName = files[i];
            double curMapTime = atof(findIndex(findIndex(curMapName, "th_keyframe"), "_").c_str());
            int posindex = find_mappos(curMapTime);
            if (i == 0)
            {
                initIndex = posindex;
                quat2euler(mappos_load[posindex].pose.orientation, initRoll, initPitch, initHeading);
                cout << "1th loaded frame's initHeading: " << initHeading << endl;
                for (int n = 0; n < loadRawCloudKeyFrame->points.size(); n++)
                {
                    double px = loadRawCloudKeyFrame->points[n].x;
                    double py = loadRawCloudKeyFrame->points[n].y;

                    double xInCar = py * cos(3.4 / 180 * M_PI) -
                             px * sin(3.4 / 180 * M_PI) +
                             0.0;
                    double yInCar = -px * cos(3.4 / 180 * M_PI) -
                             py * sin(3.4 / 180 * M_PI) +
                             0.48;
                    loadRawCloudKeyFrame->points[n].x = xInCar;
                    loadRawCloudKeyFrame->points[n].y = yInCar;
                }
                *keyframeMap += *loadRawCloudKeyFrame;
            }
            else
            {
                // transformToFirst
                double diffX = mappos_load[posindex].pose.position.x -
                                mappos_load[initIndex].pose.position.x;
                double diffY = mappos_load[posindex].pose.position.y -
                                mappos_load[initIndex].pose.position.y;
                // double deltaX = diffX * sin(initHeading) - diffY * cos(initHeading);
                // double deltaY = diffX * cos(initHeading) + diffY * sin(initHeading);
                double deltaX = diffX * cos(initHeading) + diffY * sin(initHeading);
                double deltaY = -diffX * sin(initHeading) + diffY * cos(initHeading);

                double roll_gps, pitch_gps, yaw_gps;
                quat2euler(mappos_load[posindex].pose.orientation, roll_gps, pitch_gps, yaw_gps);
                // cout << i+1 << "th loaded frame's Heading: " << yaw_gps << endl;
                double diffHeading = yaw_gps - initHeading;

                for (int n = 0; n < loadRawCloudKeyFrame->points.size(); n++)
                {
                    double xInCar = loadRawCloudKeyFrame->points[n].y * cos(3.4 / 180 * M_PI) -
                             loadRawCloudKeyFrame->points[n].x * sin(3.4 / 180 * M_PI) +
                             0.0;
                    double yInCar = -loadRawCloudKeyFrame->points[n].x * cos(3.4 / 180 * M_PI) -
                             loadRawCloudKeyFrame->points[n].y * sin(3.4 / 180 * M_PI) +
                             0.48;
                    double px = xInCar * cos(diffHeading) -
                                yInCar * sin(diffHeading) + deltaX;
                    double py = xInCar * sin(diffHeading) +
                                yInCar * cos(diffHeading) + deltaY;
                    loadRawCloudKeyFrame->points[n].x = px;
                    loadRawCloudKeyFrame->points[n].y = py;
                }
                *keyframeMap += *loadRawCloudKeyFrame;
            }
        }
        pcl::io::savePCDFileASCII("/home/ubuwgb/keyframeMap.pcd", *keyframeMap);
        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*keyframeMap, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time::now();
        cloudMsgTemp.header.frame_id = "/velodyne";
        pubKeyframeMap.publish(cloudMsgTemp);
    }

    void loadpremap()
    {
        {
            std::lock_guard<std::mutex> lock_1(mtx_cv);
            files = getFiles(SceneFolder);
            for (int i = 0; i < MapFrameNum; i++)
            {
                cout << files[i] << " " << atof(findIndex(findIndex(files[i], "th_keyframe"), "_").c_str()) << endl;
            }
            std::string pcdpath;
            cout << "loading... please wait" << pcdpath << endl;

            double time, height, qx, qy, qz, qw, lat, lon;

            for (int i = 0; i < MapFrameNum; i++)
            {
                pcdpath = SceneFolder + files[i];
                pcl::PointCloud<PointType>::Ptr loadRawCloudKeyFrame(new pcl::PointCloud<PointType>());
                pcl::io::loadPCDFile(pcdpath, *loadRawCloudKeyFrame);
                // cout << "load " << pcdpath << endl;
                scManager.makeAndSaveScancontextAndKeys(*loadRawCloudKeyFrame);
                if (i % 50 == 0)
                {
                    cout << "load " << pcdpath << endl;
                }
            }
            std::ifstream loadMapPos;
            loadMapPos.open(mapPospath.c_str());
            // cout << "loadMapPos.eof(): " << loadMapPos.eof() << endl;
            // cout << "loadMapPos.good(): " << loadMapPos.good() << endl;
            while (!loadMapPos.eof() && loadMapPos.good())
            {
                geometry_msgs::PoseStamped mappos;
                char buf[1000];
                loadMapPos.getline(buf, 1000);
                sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &time, &lat, &lon, &height, &qx, &qy, &qz, &qw);
                mappos.header.stamp.fromSec(time);
                // cout << "loading time: " << mappos.header.stamp.toSec() << endl;

                LonLat2UTM(lon, lat, mappos.pose.position.x, mappos.pose.position.y);
                // mappos.pose.position.x = lat;
                // mappos.pose.position.y = lon;
                mappos.pose.position.z = height;
                mappos.pose.orientation.x = qx;
                mappos.pose.orientation.y = qy;
                mappos.pose.orientation.z = qz;
                mappos.pose.orientation.w = qw;
                mappos_load.emplace_back(mappos);
            }
            loadMapPos.close();
            cout << MapFrameNum << " premap with "
                      << mappos_load.size() << " position is loaded.   start combine... " << endl;

            combineKeyframeMap();
            cout << "premap is combined.   start load ground truth... " << endl;
            
            std::ifstream loadgt;
            loadgt.open(gtpath.c_str());
            while (!loadgt.eof() && loadgt.good())
            {
                geometry_msgs::PoseStamped gtpose;
                char buf[1000];
                loadgt.getline(buf, 1000);
                sscanf(buf, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &time, &lat, &lon, &height, &qx, &qy, &qz, &qw);
                gtpose.header.stamp.fromSec(time);
                // cout << "loading time: " << gtpose.header.stamp.toSec() << endl;
                gtpose.pose.position.x = lat;
                gtpose.pose.position.y = lon;
                gtpose.pose.position.z = height;
                gtpose.pose.orientation.x = qx;
                gtpose.pose.orientation.y = qy;
                gtpose.pose.orientation.z = qz;
                gtpose.pose.orientation.w = qw;
                gt_load.emplace_back(gtpose);
            }
            loadgt.close();
            cout << gt_load.size() << " ground truth is loaded.   start localize... " << endl;
            premap_processed = true;
        }
        cv.notify_one();
    }

    int find_mappos(double frametime, bool find_neighbor = false)
    {
        if (!findPOS_plus)
        {
            for (int i = 0; i < mappos_load.size() - 1; i++)
            {
                if (abs(mappos_load[i].header.stamp.toSec() - frametime) < abs(mappos_load[i + 1].header.stamp.toSec() - frametime))
                {
                    // cout << i << "th diff0!: \t" << abs(mappos_load[i].header.stamp.toSec() - frametime) << endl;
                    // cout << i + 1 << "th diff0!: \t" << abs(mappos_load[i + 1].header.stamp.toSec() - frametime) << endl;
                    findPOS_plus = i;
                    break;
                }
            }
        }
        else if (find_neighbor)
        {
            // for (int i = (findPOS_plus - 300 >= 0 ? findPOS_plus - 300 : 0); i < mappos_load.size() - 1; i++)
            for (int i = 0; i < mappos_load.size() - 1; i++)
            {
                if (abs(mappos_load[i].header.stamp.toSec() - frametime) < abs(mappos_load[i + 1].header.stamp.toSec() - frametime))
                {
                    // cout << i << "th diff0!: \t" << abs(mappos_load[i].header.stamp.toSec() - frametime) << endl;
                    // cout << i + 1 << "th diff0!: \t" << abs(mappos_load[i + 1].header.stamp.toSec() - frametime) << endl;
                    return i;
                }
            }
        }
        else
        {
            for (int i = 0; i < mappos_load.size() - 1; i++)
            {
                if (abs(mappos_load[i].header.stamp.toSec() - frametime) < abs(mappos_load[i + 1].header.stamp.toSec() - frametime))
                {
                    // cout << i << "th diff: \t" << abs(mappos_load[i].header.stamp.toSec() - frametime) << endl;
                    // cout << i + 1 << "th diff: \t" << abs(mappos_load[i + 1].header.stamp.toSec() - frametime) << endl;
                    findPOS_plus = i;
                    break;
                }
            }
        }

        if (abs(mappos_load[findPOS_plus].header.stamp.toSec() - frametime) > 0.02)
        {
            cout << "[find_mappos] warning: frametime: \t" << frametime << endl;
            cout << "[find_mappos] warning: postime: \t" << mappos_load[findPOS_plus].header.stamp.toSec() << endl;
            perror("[find_mappos] time diff too large");
            // exit(1);
        }
        return findPOS_plus;
    }

    void find_gt(double pctime, double &gt_x, double &gt_y)
    {
        if (!findGT_plus)
        {
            for (int i = 0; i < gt_load.size() - 1; i++)
            {
                if (abs(gt_load[i].header.stamp.toSec() - pctime) < abs(gt_load[i + 1].header.stamp.toSec() - pctime))
                {
                    // cout << i << "th diff0!: \t" << abs(gt_load[i].header.stamp.toSec() - pctime) << endl;
                    // cout << i + 1 << "th diff0!: \t" << abs(gt_load[i + 1].header.stamp.toSec() - pctime) << endl;
                    findGT_plus = i;
                    break;
                }
            }
        }
        else
        {
            for (int i = findGT_plus; i < gt_load.size() - 1; i++)
            {
                if (abs(gt_load[i].header.stamp.toSec() - pctime) < abs(gt_load[i + 1].header.stamp.toSec() - pctime))
                {
                    // cout << i << "th diff: \t" << abs(gt_load[i].header.stamp.toSec() - pctime) << endl;
                    // cout << i + 1 << "th diff: \t" << abs(gt_load[i + 1].header.stamp.toSec() - pctime) << endl;
                    findGT_plus = i;
                    break;
                }
            }
        }
        geometry_msgs::PoseStamped gtpose = gt_load[findGT_plus];
        LonLat2UTM(gtpose.pose.position.y, gtpose.pose.position.x, gt_x, gt_y);

        if (abs(gt_load[findGT_plus].header.stamp.toSec() - pctime) > 0.02)
        {
            cout << "[find_gt] warning: abs(gt_load[" << findGT_plus << "].header.stamp.toSec() - pctime): \t"
                 << abs(gt_load[findGT_plus].header.stamp.toSec() - pctime) << endl;
            cout << "[find_gt] warning: pctime: \t" << pctime << endl;
            cout << "[find_gt] warning: gttime: \t" << gt_load[findGT_plus].header.stamp.toSec() << endl;
            perror("[find_gt] time diff too large");
            // exit(1);
        }
    }

    void laserCloudRawHandler(const sensor_msgs::PointCloud2ConstPtr &msg)
    {
        if (!LocalizeFlag)
            return;
        // cout << "handler: enter LocalizeFlag segment" << endl;
        laserCloudRawTime = msg->header.stamp.toSec();
        laserCloudRaw->clear();
        pcl::fromROSMsg(*msg, *laserCloudRaw);
        laserCloudRawDS->clear();
        downSizeFilterScancontext.setInputCloud(laserCloudRaw);
        downSizeFilterScancontext.filter(*laserCloudRawDS);
        if (generateMap)
        {
            // cout << "handler: enter generateMap segment" << endl;
            if (saveRelocalKeyFrame)
            {
                std::string rawDS_path = SceneFolder;
                rawDS_path += to_string(laserCloudRawTime) + '_';
                rawDS_path += to_string(save_startind) + "th_keyframe" +  + ".pcd";
                cout << "handler: enter saveRelocalKeyFrame segment and save pcd at: ";
                cout << rawDS_path << endl;
                pcl::io::savePCDFileASCII(rawDS_path, *laserCloudRawDS);
                save_startind++;
                pcdStillNotSave = false;
                saveRelocalKeyFrame = false;
            }
            return;
        }
        pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudRawDS, *thisRawCloudKeyFrame);
        relocalNum = relocalNum + relocalHz;
        if (relocalNum >= 10)
        {
            relocalNum = 0;
            std::unique_lock<std::mutex> unilock(mtx_cv);
            cv.wait(unilock, [&] { return premap_processed; });
            cout << laserCloudRawTime << ", start detectRelocalID ..." << endl;
            auto localizeResult = scManager.detectRelocalID(*thisRawCloudKeyFrame);
            sci_localized = true;
            localizeResultIndex = localizeResult.first;
            YawDiff = localizeResult.second;
            cout << "------ localizeResult.Index: " << localizeResultIndex;
            cout << "------ localizeResult.YawDiff: " << YawDiff << endl;
            unilock.unlock();

            // ++relocalSaveInd;

            // std::string relocalPcdPath = getenv("HOME");
            // relocalPcdPath += "/catkin_ws/data/localizeFrame/";
            // relocalPcdPath += to_string(relocalSaveInd) + "th_keyframe_";
            // relocalPcdPath += to_string(laserCloudRawTime) + ".pcd";
            // pcl::io::savePCDFileASCII(relocalPcdPath, *thisRawCloudKeyFrame);

            if (localizeResultIndex != -1)
            {
                std::string resultName = files[localizeResultIndex];
                double resultTime = atof(findIndex(findIndex(resultName, "th_keyframe"), "_").c_str());
                int posindex = find_mappos(resultTime);
                pcl::PointCloud<PointType>::Ptr resultpcd(new pcl::PointCloud<PointType>());

                // a. single registration
                // std::string resultpath = SceneFolder + files[localizeResultIndex];
                // pcl::io::loadPCDFile(resultpath, *resultpcd);

                // b. multi-frame registration
                pcl::PointCloud<PointType>::Ptr oneframe(new pcl::PointCloud<PointType>());
                double initRoll, initPitch, initHeading;
                quat2euler(mappos_load[posindex].pose.orientation, initRoll, initPitch, initHeading);
                for (int i = localizeResultIndex - 3; i <= localizeResultIndex + 3; i++)
                {
                    if (i < 0 || i >= files.size())
                        continue;
                    oneframe->clear();
                    std::string resultpath = SceneFolder + files[i];
                    pcl::io::loadPCDFile(resultpath, *oneframe);
                    double neighborTime = atof(findIndex(findIndex(files[i], "th_keyframe"), "_").c_str());
                    int neighborindex = find_mappos(neighborTime, true);

                    // transformToFirst
                    double diffX = mappos_load[neighborindex].pose.position.x -
                                    mappos_load[posindex].pose.position.x;
                    double diffY = mappos_load[neighborindex].pose.position.y -
                                    mappos_load[posindex].pose.position.y;
                    double deltaX = diffX * cos(initHeading) + diffY * sin(initHeading);
                    double deltaY = -diffX * sin(initHeading) + diffY * cos(initHeading);

                    double roll_gps, pitch_gps, yaw_gps;
                    quat2euler(mappos_load[neighborindex].pose.orientation, roll_gps, pitch_gps, yaw_gps);
                    // cout << i+1 << "th loaded frame's Heading: " << yaw_gps << endl;
                    double diffHeading = yaw_gps - initHeading;

                    for (int n = 0; n < oneframe->points.size(); n++)
                    {
                        double xInCar = oneframe->points[n].y * cos(3.4 / 180 * M_PI) -
                                oneframe->points[n].x * sin(3.4 / 180 * M_PI) +
                                0.0;
                        double yInCar = -oneframe->points[n].x * cos(3.4 / 180 * M_PI) -
                                oneframe->points[n].y * sin(3.4 / 180 * M_PI) +
                                0.48;
                        double px = xInCar * cos(diffHeading) -
                                    yInCar * sin(diffHeading) + deltaX;
                        double py = xInCar * sin(diffHeading) +
                                    yInCar * cos(diffHeading) + deltaY;
                        double px2 = (0.48 - py) * cos(3.4 / 180 * M_PI) - px * sin(3.4 / 180 * M_PI);
                        double py2 = (0.48 - py) * sin(3.4 / 180 * M_PI) + px * cos(3.4 / 180 * M_PI);
                        oneframe->points[n].x = px2;
                        oneframe->points[n].y = py2;
                    }
                    *resultpcd += *oneframe;
                }
                
                // cout << "[GPS evaluation] resultpcd's path: " << resultpath << endl;

                // get gt position and sparse-estimation
                double gt_x, gt_y, relocal_x, relocal_y;
                find_gt(laserCloudRawTime, gt_x, gt_y);
                relocal_x = mappos_load[posindex].pose.position.x;
                relocal_y = mappos_load[posindex].pose.position.y;

                if (sqrt((gt_x - relocal_x)*(gt_x - relocal_x)+(gt_y - relocal_y)*(gt_y - relocal_y)) >= 8 )
                {
                    cout << "[GPS validation] this frame is mismatch!!!" << endl;
                    // double mindist = 100;
                    // int minIndex = 1;
                    // for(auto it2 = utmRecord_load.begin(); it2 != utmRecord_load.end(); ++it2)
                    // {
                    //     double dist = (utm_EN.first - it2->first)*(utm_EN.first - it2->first)+(utm_EN.second - it2->second)*(utm_EN.second - it2->second);
                    //     if (dist < mindist)
                    //     {
                    //         mindist = dist;
                    //         minIndex = (it2 - utmRecord_load.begin()) + 1;
                    //     }
                    // }   
                    // ++mismatchNumber;
                    // cout << "[GPS validation] currentFrameID: " << ++currentFrameID
                    //      << "     failNumber: " << failNumber
                    //      << "     mismatchNumber: " << mismatchNumber << "------" << endl;
                    // std::ofstream gpsFail;
                    // gpsFail.open(gpsFailPath, std::ios::app);
                    // int FailFrameID = currentFrameID + 1;
                    // gpsFail<<std::setprecision(15)<<utm_EN.first<<" "<<utm_EN.second<<" "<<FailFrameID<<" "<<misFlag<<" "<<minIndex<<endl;
                    // gpsFail.close();
                }

                cout << std::setprecision(15) << "[GPS evaluation] relocal-frame GPS: "
                     << relocal_x << ", " << relocal_y << endl;
                cout << "[GPS evaluation] current GPS:       " << gt_x << ", " << gt_y << endl;
                
                Eigen::Affine3f relocal_tune;
                float x, y, z, roll, pitch, yaw;

                // pcl::NormalDistributionsTransform<PointType, PointType> ndt_relocal;
                // // Setting scale dependent NDT parameters
                // // Setting minimum transformation difference for termination condition.
                // ndt_relocal.setTransformationEpsilon (0.01);
                // // Setting maximum step size for More-Thuente line search.
                // ndt_relocal.setStepSize (0.1);
                // //Setting Resolution of NDT grid structure (VoxelGridCovariance).
                // ndt_relocal.setResolution (1.0);
                // // Setting max number of registration iterations.
                // ndt_relocal.setMaximumIterations (35);
                // // Setting point cloud to be aligned.
                // ndt_relocal.setInputSource (thisRawCloudKeyFrame);
                // // Setting point cloud to be aligned to.
                // ndt_relocal.setInputTarget (resultpcd);
                // pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
                // ndt_relocal.align(*unused_result);
                // cout << "[GPS evaluation] NDT fit score: " << ndt_relocal.getFitnessScore() << endl;
                // if (ndt_relocal.hasConverged() == false || ndt_relocal.getFitnessScore() > historyKeyframeFitnessScore)
                // {
                //     cout << "[GPS evaluation] warning: bad NDT fit score, > " << historyKeyframeFitnessScore << endl;
                // }
                // if (ndt_relocal.hasConverged() == true)
                // {
                //     relocal_tune = ndt_relocal.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)

                pcl::PassThrough<PointType> pass;
                pass.setInputCloud(thisRawCloudKeyFrame);
                pass.setFilterFieldName("z");
                pass.setFilterLimits(-1.2, 20);
                //pass.setFilterLimitsNegative (true);
                pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame_filtered(new pcl::PointCloud<PointType>());
                pass.filter(*thisRawCloudKeyFrame_filtered);

                pass.setInputCloud(resultpcd);
                //pass.setFilterLimitsNegative (true);
                pcl::PointCloud<PointType>::Ptr resultpcd_filtered(new pcl::PointCloud<PointType>());
                pass.filter(*resultpcd_filtered);

                pcl::IterativeClosestPoint<PointType, PointType> icp_relocal;
                icp_relocal.setMaxCorrespondenceDistance(100);
                icp_relocal.setMaximumIterations(100);
                icp_relocal.setTransformationEpsilon(1e-6);
                icp_relocal.setEuclideanFitnessEpsilon(1e-6);
                icp_relocal.setRANSACIterations(0);

                // Align clouds
                // Eigen::Affine3f icpInitialMatFoo = pcl::getTransformation(0, 0, 0, yawDiffRad, 0, 0); // because within cam coord: (z, x, y, yaw, roll, pitch)
                // Eigen::Matrix4f icpInitialMat = icpInitialMatFoo.matrix();
                icp_relocal.setInputSource(thisRawCloudKeyFrame_filtered);
                icp_relocal.setInputTarget(resultpcd_filtered);
                pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
                icp_relocal.align(*unused_result);
                // icp_relocal.align(*unused_result, icpInitialMat); // PCL icp non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)

                cout << "[GPS evaluation] ICP fit score: " << icp_relocal.getFitnessScore() << endl;
                if (icp_relocal.hasConverged() == false || icp_relocal.getFitnessScore() > historyKeyframeFitnessScore)
                {
                    perror("icp_relocal is bad");
                    cout << "[GPS evaluation] warning: icp_relocal.hasConverged(): " << icp_relocal.hasConverged() << endl;
                    cout << "[GPS evaluation] warning: icp fit score : " << icp_relocal.getFitnessScore()
                         << ", historyKeyframeFitnessScore : " << historyKeyframeFitnessScore << endl;
                }
                if (icp_relocal.hasConverged() == true)
                {
                    relocal_tune = icp_relocal.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                    Eigen::Affine3f lidar2car;

                    float A = -sin(3.4 / 180 * M_PI),  B = cos (3.4 / 180 * M_PI);

                    lidar2car (0, 0) = A;   lidar2car (0, 1) = B;   lidar2car (0, 2) = 0;   lidar2car (0, 3) = 0;
                    lidar2car (1, 0) = -B;  lidar2car (1, 1) = A;   lidar2car (1, 2) = 0;   lidar2car (1, 3) = 0.48;
                    lidar2car (2, 0) = 0;   lidar2car (2, 1) = 0;   lidar2car (2, 2) = 1;   lidar2car (2, 3) = 0;
                    lidar2car (3, 0) = 0;   lidar2car (3, 1) = 0;   lidar2car (3, 2) = 0;   lidar2car (3, 3) = 1;
                    // cout << "test matrix *: " << (lidar2car.matrix()) * (lidar2car.inverse().matrix()) << endl;
                    Eigen::Matrix4f middle_tuneMat = (relocal_tune.matrix()) * (lidar2car.inverse().matrix());
                    Eigen::Matrix4f car_tuneMat =  (lidar2car.matrix()) * middle_tuneMat;
                    Eigen::Affine3f car_tune;
                    car_tune = car_tuneMat;
                    // pcl::getTranslationAndEulerAngles(relocal_tune.inverse(), x, y, z, roll, pitch, yaw);
                    pcl::getTranslationAndEulerAngles(car_tune, x, y, z, roll, pitch, yaw);
                    cout << "[GPS evaluation] The icp.trans is \n" << relocal_tune.matrix() << endl;
                    cout << "[GPS evaluation] The car_tune is \n" << car_tune.matrix() << endl;
                    cout << "[GPS evaluation] The fine-tune result is "
                         << "\n\tx:    " << x << "\ty:   " << y << "\tz:     " << z
                         << "\n\troll: " << roll << "\tyaw: " << yaw << "\tpitch: " << pitch << endl;
                    double roll_gps, pitch_gps, yaw_gps;
                    quat2euler(mappos_load[posindex].pose.orientation, roll_gps, pitch_gps, yaw_gps);
                    cout << "[GPS evaluation] The detected relocal frame's rpy is \n\t"
                        //  << "roll: " << roll_gps << "\n\t"
                        //  << "pitch: " << pitch_gps << "\n\t"
                        //  << "yaw: " << yaw_gps << "\n\t"
                         << "heading: " << yaw_gps << endl;
                    double heading = yaw_gps;
                    // relocal_x = relocal_x - x * sin(heading) - y * cos(heading);
                    // relocal_y = relocal_y + x * cos(heading) - y * sin(heading);
                    relocal_x = relocal_x + x * cos(heading) - y * sin(heading);
                    relocal_y = relocal_y + x * sin(heading) + y * cos(heading);
                    cout << "[GPS evaluation] The relocal fine-tune GPS:\n\t"
                         << "relocal_x: " << relocal_x << "\t"
                         << "relocal_y: " << relocal_y << "\n\t"
                         << "diff_x:    " << relocal_x - gt_x << "\t"
                         << "diff_y:    " << relocal_y - gt_y << endl;
                    if (icp_relocal.getFitnessScore() < historyKeyframeFitnessScore)
                    {
                        evaluationTxt << std::setprecision(15)
                                      << laserCloudRawTime
                                      //   << "laserTime: " << laserCloudRawTime
                                      //   << ", " << gt_x
                                      //   << " " << gt_y << ", "
                                      << " " << relocal_x - gt_x
                                      << " " << relocal_y - gt_y
                                      << " " << (YawDiff * 180.0 / M_PI)
                                      << " " << x << " " << y
                                      << " icp.fitok" << endl;
                    }
                    else
                    {
                        evaluationTxt << std::setprecision(15)
                                      << laserCloudRawTime
                                      //   << "laserTime: " << laserCloudRawTime
                                      //   << ", " << gt_x
                                      //   << " " << gt_y << ", "
                                      << " " << relocal_x - gt_x
                                      << " " << relocal_y - gt_y
                                      << " " << (YawDiff * 180.0 / M_PI)
                                      << " " << x << " " << y
                                      << " icp.fitbad" << endl;
                    }
                }
                else
                {
                    cout << "[GPS evaluation] The relocal non-tune GPS:\n\t"
                              << "relocal_x: " << relocal_x << "\n\t"
                              << "relocal_y: " << relocal_y << "\n\t"
                              << "diff_x:    " << relocal_x - gt_x << "\n\t"
                              << "diff_y:    " << relocal_y - gt_y << endl;
                    evaluationTxt << std::setprecision(15)
                                  << laserCloudRawTime
                                  //   << "laserTime: " << laserCloudRawTime
                                  //   << ", " << gt_x
                                  //   << " " << gt_y << ", "
                                  << " " << relocal_x - gt_x
                                  << " " << relocal_y - gt_y
                                  << " noicp" << endl;
                }
                cout << "\n\n";
                // for (int i = 0; i < resultpcd->points.size(); i++)
                // {
                //     auto x = resultpcd->points[i].x * cos(YawDiff) - resultpcd->points[i].y * sin(YawDiff);
                //     auto y = resultpcd->points[i].x * sin(YawDiff) + resultpcd->points[i].y * cos(YawDiff);
                //     resultpcd->points[i].x = x;
                //     resultpcd->points[i].y = y;
                //     resultpcd->points[i].z += 20;
                // }

                {
                    sensor_msgs::PointCloud2 cloudMsgTemp;
                    pcl::toROSMsg(*resultpcd_filtered, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
                    cloudMsgTemp.header.frame_id = "/velodyne";
                    pubRelocalTarget.publish(cloudMsgTemp);
                }
                {
                    sensor_msgs::PointCloud2 cloudMsgTemp;
                    pcl::toROSMsg(*thisRawCloudKeyFrame_filtered, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
                    cloudMsgTemp.header.frame_id = "/velodyne";
                    pubRelocalSource.publish(cloudMsgTemp);
                }
                {
                    sensor_msgs::PointCloud2 cloudMsgTemp;
                    pcl::toROSMsg(*unused_result, cloudMsgTemp);
                    cloudMsgTemp.header.stamp = ros::Time().fromSec(laserCloudRawTime);
                    cloudMsgTemp.header.frame_id = "/velodyne";
                    pubRelocalICP.publish(cloudMsgTemp);
                }

                /**
----------------------------------------transform------------------------------------------------------
                for (int trans = 0; trans < 21; ++trans)
                {
                    pcl::PointCloud<PointType>::Ptr copy(new pcl::PointCloud<PointType>());
                    pcl::copyPointCloud(*resultpcd,  *copy);
                for (int i = 0; i < copy->points.size(); i++)
                {
                    copy->points[i].y -= trans;
                    copy->points[i].z -= 20;
                }

                sensor_msgs::PointCloud2 transformCloud;
                pcl::toROSMsg(*copy, transformCloud);
                transformCloud.header.stamp = ros::Time().fromSec(laserCloudRawTime);
                transformCloud.header.frame_id = "/velodyne";
                pubTransformCloud.publish(transformCloud);

                Eigen::MatrixXd sc = forTransform.makeScancontext(*copy);
                cv::Mat sciForRedPoint = forTransform.createSci(sc);
                char name[100];
                sprintf(name,"transformPoint_y+%d",trans);
                cv::Mat sciForRedPointAddAxes = forTransform.addAxes(sciForRedPoint,name);
                cv::imshow("transform point",sciForRedPointAddAxes);
                cv::waitKey(1);
                sleep(1);
               
                }
----------------------------------------transform------------------------------------------------------
**/
            }
            else
            {
                cout << "[GPS validation] this frame is failed!!!" << endl;
                // double mindist = 100;
                // int minIndex = 1;
                // for(auto it2 = utmRecord_load.begin(); it2 != utmRecord_load.end(); ++it2)
                // {
                //     double dist = (utm_EN.first - it2->first)*(utm_EN.first - it2->first)+(utm_EN.second - it2->second)*(utm_EN.second - it2->second);
                //     if (dist < mindist)
                //     {
                //         mindist = dist;
                //         minIndex = (it2 - utmRecord_load.begin()) + 1;
                //     }
                // }               
                // std::ofstream gpsFail;
                // gpsFail.open(gpsFail_path, std::ios::app);
                // int FailFrameID = currentFrameID + 1;
                // gpsFail<<std::setprecision(15)<<utm_EN.first<<" "<<utm_EN.second<<" "<<FailFrameID<<" "<<failFlag<<" "<<minIndex<<endl;
                // gpsFail.close();
                // cout << "------" << "currentFrameID: " << ++currentFrameID << "     failNumber: " << ++failNumber << "     mismatchNumber: "<< mismatchNumber << "------"<<endl;
            }
            // cv.notify_one();
        }
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok())
        {
            rate.sleep();
            publishGlobalMap();
        }
        // save final point cloud
        pcl::io::savePCDFileASCII(fileDirectory + "finalCloud.pcd", *globalMapKeyFramesDS);

        string cornerMapString = "/tmp/cornerMap.pcd";
        string surfaceMapString = "/tmp/surfaceMap.pcd";
        string trajectoryString = "/tmp/trajectory.pcd";

        pcl::PointCloud<PointType>::Ptr cornerMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr cornerMapCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surfaceMapCloudDS(new pcl::PointCloud<PointType>());

        for (int i = 0; i < cornerCloudKeyFrames.size(); i++)
        {
            *cornerMapCloud += *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *surfaceMapCloud += *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *surfaceMapCloud += *transformPointCloud(outlierCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
        }

        downSizeFilterCorner.setInputCloud(cornerMapCloud);
        downSizeFilterCorner.filter(*cornerMapCloudDS);
        downSizeFilterSurf.setInputCloud(surfaceMapCloud);
        downSizeFilterSurf.filter(*surfaceMapCloudDS);

        pcl::io::savePCDFileASCII(fileDirectory + "cornerMap.pcd", *cornerMapCloudDS);
        pcl::io::savePCDFileASCII(fileDirectory + "surfaceMap.pcd", *surfaceMapCloudDS);
        pcl::io::savePCDFileASCII(fileDirectory + "trajectory.pcd", *cloudKeyPoses3D);
    }

    // bool load_map_flag = true;
    void publishGlobalMap()
    {
        // if(load_map_flag){
        // std::lock_guard<std::mutex> lock_1(mtx_cv);
        // files = getFiles(SceneFolder);
        // std::string pcdpath;
        // cout << "loading... please wait" << pcdpath << endl;

        // FILE *gpsIn;
        // std::pair<double,double> utm_EN;
        // double time, utm_e, utm_n, height, qx, qy, qz, qw;
        // gpsIn = fopen(gpstxt_path.c_str(),"r");
        // int i = 1;
        // while(i < save_startind)
        // {
        //     fscanf(gpsIn, "%lf %lf %lf %lf %lf %lf %lf %lf\n", &utm_e, &utm_n, &height, &qx, &qy, &qz, &qw, &time);
        //     utm_EN.first = utm_e;
        //     utm_EN.second = utm_n;
        //     if (utm_e == 0 && utm_n == 0)
        //     {
        //         ++i;
        //         continue;
        //     }
        //     scManager.utmRecord.push_back(utm_EN);
        //     ++i;

        // }
        // cout << "loading finish" << pcdpath << endl;
        // load_map_flag = false;
        // cv.notify_one();
        // }

        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;

        if (cloudKeyPoses3D->points.empty() == true)
            return;
        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(currentRobotPosPoint, globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();

        for (int i = 0; i < pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        // extract visualized and downsampled key frames
        for (int i = 0; i < globalMapKeyPosesDS->points.size(); ++i)
        {
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(outlierCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);

        sensor_msgs::PointCloud2 cloudMsgTemp;
        pcl::toROSMsg(*globalMapKeyFramesDS, cloudMsgTemp);
        cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        cloudMsgTemp.header.frame_id = "/camera_init";
        pubLaserCloudSurround.publish(cloudMsgTemp);

        globalMapKeyPoses->clear();
        globalMapKeyPosesDS->clear();
        globalMapKeyFrames->clear();
        // globalMapKeyFramesDS->clear();
    }

    void loopClosureThread()
    {

        if (loopClosureEnableFlag == false)
            return;

        ros::Rate rate(1);
        while (ros::ok())
        {
            rate.sleep();
            performLoopClosure();
        }
    } // loopClosureThread

    bool detectLoopClosure()
    {

        std::lock_guard<std::mutex> lock(mtx);

        /* 
         * 1. xyz distance-based radius search (contained in the original LeGO LOAM code)
         * - for fine-stichting trajectories (for not-recognized nodes within scan context search) 
         */
        RSlatestSurfKeyFrameCloud->clear();
        RSnearHistorySurfKeyFrameCloud->clear();
        RSnearHistorySurfKeyFrameCloudDS->clear();

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(cloudKeyPoses3D);
        kdtreeHistoryKeyPoses->radiusSearch(currentRobotPosPoint, historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);

        RSclosestHistoryFrameID = -1;
        int curMinID = 1000000;
        // policy: take Oldest one (to fix error of the whole trajectory)
        for (int i = 0; i < pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(cloudKeyPoses6D->points[id].time - timeLaserOdometry) > 30.0)
            {
                // RSclosestHistoryFrameID = id;
                // break;
                if (id < curMinID)
                {
                    curMinID = id;
                    RSclosestHistoryFrameID = curMinID;
                }
            }
        }

        if (RSclosestHistoryFrameID == -1)
        {
            // Do nothing here
            // then, do the next check: Scan context-based search
            // not return false here;
        }
        else
        {
            // save latest key frames
            latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
            *RSlatestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
            *RSlatestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
            pcl::PointCloud<PointType>::Ptr RShahaCloud(new pcl::PointCloud<PointType>());
            int cloudSize = RSlatestSurfKeyFrameCloud->points.size();
            for (int i = 0; i < cloudSize; ++i)
            {
                if ((int)RSlatestSurfKeyFrameCloud->points[i].intensity >= 0)
                {
                    RShahaCloud->push_back(RSlatestSurfKeyFrameCloud->points[i]);
                }
            }
            RSlatestSurfKeyFrameCloud->clear();
            *RSlatestSurfKeyFrameCloud = *RShahaCloud;

            // save history near key frames
            for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j)
            {
                if (RSclosestHistoryFrameID + j < 0 || RSclosestHistoryFrameID + j > latestFrameIDLoopCloure)
                    continue;
                *RSnearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[RSclosestHistoryFrameID + j], &cloudKeyPoses6D->points[RSclosestHistoryFrameID + j]);
                *RSnearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[RSclosestHistoryFrameID + j], &cloudKeyPoses6D->points[RSclosestHistoryFrameID + j]);
            }
            downSizeFilterHistoryKeyFrames.setInputCloud(RSnearHistorySurfKeyFrameCloud);
            downSizeFilterHistoryKeyFrames.filter(*RSnearHistorySurfKeyFrameCloudDS);
        }

        /* 
         * 2. Scan context-based global localization 
         */
        SClatestSurfKeyFrameCloud->clear();
        SCnearHistorySurfKeyFrameCloud->clear();
        SCnearHistorySurfKeyFrameCloudDS->clear();

        // std::lock_guard<std::mutex> lock(mtx);
        latestFrameIDLoopCloure = cloudKeyPoses3D->points.size() - 1;
        SCclosestHistoryFrameID = -1;                        // init with -1
        auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
        SCclosestHistoryFrameID = detectResult.first;
        yawDiffRad = detectResult.second; // not use for v1 (because pcl icp withi initial somthing wrong...)

        // if all close, reject
        if (SCclosestHistoryFrameID == -1)
        {
            return false;
        }

        // save latest key frames: query ptcloud (corner points + surface points)
        // NOTE: using "closestHistoryFrameID" to make same root of submap points to get a direct relative between the query point cloud (latestSurfKeyFrameCloud) and the map (nearHistorySurfKeyFrameCloud). by giseop
        // i.e., set the query point cloud within mapside's local coordinate
        *SClatestSurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[SCclosestHistoryFrameID]);
        *SClatestSurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[latestFrameIDLoopCloure], &cloudKeyPoses6D->points[SCclosestHistoryFrameID]);

        pcl::PointCloud<PointType>::Ptr SChahaCloud(new pcl::PointCloud<PointType>());
        int cloudSize = SClatestSurfKeyFrameCloud->points.size();
        for (int i = 0; i < cloudSize; ++i)
        {
            if ((int)SClatestSurfKeyFrameCloud->points[i].intensity >= 0)
            {
                SChahaCloud->push_back(SClatestSurfKeyFrameCloud->points[i]);
            }
        }
        SClatestSurfKeyFrameCloud->clear();
        *SClatestSurfKeyFrameCloud = *SChahaCloud;

        // save history near key frames: map ptcloud (icp to query ptcloud)
        for (int j = -historyKeyframeSearchNum; j <= historyKeyframeSearchNum; ++j)
        {
            if (SCclosestHistoryFrameID + j < 0 || SCclosestHistoryFrameID + j > latestFrameIDLoopCloure)
                continue;
            *SCnearHistorySurfKeyFrameCloud += *transformPointCloud(cornerCloudKeyFrames[SCclosestHistoryFrameID + j], &cloudKeyPoses6D->points[SCclosestHistoryFrameID + j]);
            *SCnearHistorySurfKeyFrameCloud += *transformPointCloud(surfCloudKeyFrames[SCclosestHistoryFrameID + j], &cloudKeyPoses6D->points[SCclosestHistoryFrameID + j]);
        }
        downSizeFilterHistoryKeyFrames.setInputCloud(SCnearHistorySurfKeyFrameCloud);
        downSizeFilterHistoryKeyFrames.filter(*SCnearHistorySurfKeyFrameCloudDS);

        // // optional: publish history near key frames
        // if (pubHistoryKeyFrames.getNumSubscribers() != 0){
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*nearHistorySurfKeyFrameCloudDS, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        //     cloudMsgTemp.header.frame_id = "/camera_init";
        //     pubHistoryKeyFrames.publish(cloudMsgTemp);
        // }

        return true;
    } // detectLoopClosure

    void performLoopClosure(void)
    {

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        // try to find close key frame if there are any
        if (potentialLoopFlag == false)
        {
            if (detectLoopClosure() == true)
            {
                cout << endl;
                potentialLoopFlag = true; // find some key frames that is old enough or close enough for loop closure
                timeSaveFirstCurrentScanForLoopClosure = timeLaserOdometry;
            }
            if (potentialLoopFlag == false)
            {
                return;
            }
        }

        // reset the flag first no matter icp successes or not
        potentialLoopFlag = false;

        // *****
        // Main
        // *****
        // make common variables at forward
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionCameraFrame;
        float noiseScore = 0.5; // constant is ok...
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        constraintNoise = noiseModel::Diagonal::Variances(Vector6);
        robustNoiseModel = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure
            gtsam::noiseModel::Diagonal::Variances(Vector6)); // - checked it works. but with robust kernel, map modification may be delayed (i.e,. requires more true-positive loop factors)

        bool isValidRSloopFactor = false;
        bool isValidSCloopFactor = false;

        /*
         * 1. RS loop factor (radius search)
         */
        if (RSclosestHistoryFrameID != -1)
        {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            icp.setInputSource(RSlatestSurfKeyFrameCloud);
            icp.setInputTarget(RSnearHistorySurfKeyFrameCloudDS);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);

            cout << "[RS] ICP fit score: " << icp.getFitnessScore() << endl;
            if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            {
                cout << "[RS] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")" << endl;
                isValidRSloopFactor = false;
            }
            else
            {
                cout << "[RS] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure << " ] and RS nearest [ " << RSclosestHistoryFrameID << " ]" << endl;
                isValidRSloopFactor = true;
            }

            if (isValidRSloopFactor == true)
            {
                correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
                Eigen::Affine3f correctionLidarFrame = pcl::getTransformation(z, x, y, yaw, roll, pitch);
                // transform from world origin to wrong pose
                Eigen::Affine3f tWrong = pclPointToAffine3fCameraToLidar(cloudKeyPoses6D->points[latestFrameIDLoopCloure]);
                // transform from world origin to corrected pose
                Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
                pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
                gtsam::Pose3 poseTo = pclPointTogtsamPose3(cloudKeyPoses6D->points[RSclosestHistoryFrameID]);
                gtsam::Vector Vector6(6);

                std::lock_guard<std::mutex> lock(mtx);
                gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, RSclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel));
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
        }

        /*
         * 2. SC loop factor (scan context)
         */
        if (SCclosestHistoryFrameID != -1)
        {
            pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(100);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            // Eigen::Affine3f icpInitialMatFoo = pcl::getTransformation(0, 0, 0, yawDiffRad, 0, 0); // because within cam coord: (z, x, y, yaw, roll, pitch)
            // Eigen::Matrix4f icpInitialMat = icpInitialMatFoo.matrix();
            icp.setInputSource(SClatestSurfKeyFrameCloud);
            icp.setInputTarget(SCnearHistorySurfKeyFrameCloudDS);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
            // icp.align(*unused_result, icpInitialMat); // PCL icp non-eye initial is bad ... don't use (LeGO LOAM author also said pcl transform is weird.)

            cout << "[SC] ICP fit score: " << icp.getFitnessScore() << endl;
            if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            {
                cout << "[SC] Reject this loop (bad icp fit score, > " << historyKeyframeFitnessScore << ")" << endl;
                isValidSCloopFactor = false;
            }
            else
            {
                cout << "[SC] The detected loop factor is added between Current [ " << latestFrameIDLoopCloure << " ] and SC nearest [ " << SCclosestHistoryFrameID << " ]" << endl;
                isValidSCloopFactor = true;
            }

            if (isValidSCloopFactor == true)
            {
                correctionCameraFrame = icp.getFinalTransformation(); // get transformation in camera frame (because points are in camera frame)
                pcl::getTranslationAndEulerAngles(correctionCameraFrame, x, y, z, roll, pitch, yaw);
                gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
                gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

                std::lock_guard<std::mutex> lock(mtx);
                // gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, closestHistoryFrameID, poseFrom.between(poseTo), constraintNoise)); // original
                gtSAMgraph.add(BetweenFactor<Pose3>(latestFrameIDLoopCloure, SCclosestHistoryFrameID, poseFrom.between(poseTo), robustNoiseModel)); // giseop
                isam->update(gtSAMgraph);
                isam->update();
                gtSAMgraph.resize(0);
            }
        }

        // just for visualization
        // // publish corrected cloud
        // if (pubIcpKeyFrames.getNumSubscribers() != 0){
        //     pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
        //     pcl::transformPointCloud (*latestSurfKeyFrameCloud, *closed_cloud, icp.getFinalTransformation());
        //     sensor_msgs::PointCloud2 cloudMsgTemp;
        //     pcl::toROSMsg(*closed_cloud, cloudMsgTemp);
        //     cloudMsgTemp.header.stamp = ros::Time().fromSec(timeLaserOdometry);
        //     cloudMsgTemp.header.frame_id = "/camera_init";
        //     pubIcpKeyFrames.publish(cloudMsgTemp);
        // }

        // flagging
        aLoopIsClosed = true;

    } // performLoopClosure

    Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    { // camera frame to lidar frame
        return Pose3(Rot3::RzRyRx(double(thisPoint.yaw), double(thisPoint.roll), double(thisPoint.pitch)),
                     Point3(double(thisPoint.z), double(thisPoint.x), double(thisPoint.y)));
    }

    Eigen::Affine3f pclPointToAffine3fCameraToLidar(PointTypePose thisPoint)
    { // camera frame to lidar frame
        return pcl::getTransformation(thisPoint.z, thisPoint.x, thisPoint.y, thisPoint.yaw, thisPoint.roll, thisPoint.pitch);
    }

    void extractSurroundingKeyFrames()
    {

        if (cloudKeyPoses3D->points.empty() == true)
            return;

        if (loopClosureEnableFlag == true)
        {
            // only use recent key poses for graph building
            if (recentCornerCloudKeyFrames.size() < surroundingKeyframeSearchNum)
            { // queue is not full (the beginning of mapping or a loop is just closed)
                // clear recent key frames queue
                recentCornerCloudKeyFrames.clear();
                recentSurfCloudKeyFrames.clear();
                recentOutlierCloudKeyFrames.clear();
                int numPoses = cloudKeyPoses3D->points.size();
                for (int i = numPoses - 1; i >= 0; --i)
                {
                    int thisKeyInd = (int)cloudKeyPoses3D->points[i].intensity;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    // extract surrounding map
                    recentCornerCloudKeyFrames.push_front(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                    recentSurfCloudKeyFrames.push_front(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                    recentOutlierCloudKeyFrames.push_front(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                    if (recentCornerCloudKeyFrames.size() >= surroundingKeyframeSearchNum)
                        break;
                }
            }
            else
            { // queue is full, pop the oldest key frame and push the latest key frame
                if (latestFrameID != cloudKeyPoses3D->points.size() - 1)
                { // if the robot is not moving, no need to update recent frames

                    recentCornerCloudKeyFrames.pop_front();
                    recentSurfCloudKeyFrames.pop_front();
                    recentOutlierCloudKeyFrames.pop_front();
                    // push latest scan to the end of queue
                    latestFrameID = cloudKeyPoses3D->points.size() - 1;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[latestFrameID];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    recentCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[latestFrameID]));
                    recentSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[latestFrameID]));
                    recentOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[latestFrameID]));
                }
            }

            for (int i = 0; i < recentCornerCloudKeyFrames.size(); ++i)
            {
                *laserCloudCornerFromMap += *recentCornerCloudKeyFrames[i];
                *laserCloudSurfFromMap += *recentSurfCloudKeyFrames[i];
                *laserCloudSurfFromMap += *recentOutlierCloudKeyFrames[i];
            }
        }
        else
        {
            surroundingKeyPoses->clear();
            surroundingKeyPosesDS->clear();
            // extract all the nearby key poses and downsample them
            kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D);
            kdtreeSurroundingKeyPoses->radiusSearch(currentRobotPosPoint, (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis, 0);
            for (int i = 0; i < pointSearchInd.size(); ++i)
                surroundingKeyPoses->points.push_back(cloudKeyPoses3D->points[pointSearchInd[i]]);
            downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
            downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
            // delete key frames that are not in surrounding region
            int numSurroundingPosesDS = surroundingKeyPosesDS->points.size();
            for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i)
            {
                bool existingFlag = false;
                for (int j = 0; j < numSurroundingPosesDS; ++j)
                {
                    if (surroundingExistingKeyPosesID[i] == (int)surroundingKeyPosesDS->points[j].intensity)
                    {
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == false)
                {
                    surroundingExistingKeyPosesID.erase(surroundingExistingKeyPosesID.begin() + i);
                    surroundingCornerCloudKeyFrames.erase(surroundingCornerCloudKeyFrames.begin() + i);
                    surroundingSurfCloudKeyFrames.erase(surroundingSurfCloudKeyFrames.begin() + i);
                    surroundingOutlierCloudKeyFrames.erase(surroundingOutlierCloudKeyFrames.begin() + i);
                    --i;
                }
            }
            // add new key frames that are not in calculated existing key frames
            for (int i = 0; i < numSurroundingPosesDS; ++i)
            {
                bool existingFlag = false;
                for (auto iter = surroundingExistingKeyPosesID.begin(); iter != surroundingExistingKeyPosesID.end(); ++iter)
                {
                    if ((*iter) == (int)surroundingKeyPosesDS->points[i].intensity)
                    {
                        existingFlag = true;
                        break;
                    }
                }
                if (existingFlag == true)
                {
                    continue;
                }
                else
                {
                    int thisKeyInd = (int)surroundingKeyPosesDS->points[i].intensity;
                    PointTypePose thisTransformation = cloudKeyPoses6D->points[thisKeyInd];
                    updateTransformPointCloudSinCos(&thisTransformation);
                    surroundingExistingKeyPosesID.push_back(thisKeyInd);
                    surroundingCornerCloudKeyFrames.push_back(transformPointCloud(cornerCloudKeyFrames[thisKeyInd]));
                    surroundingSurfCloudKeyFrames.push_back(transformPointCloud(surfCloudKeyFrames[thisKeyInd]));
                    surroundingOutlierCloudKeyFrames.push_back(transformPointCloud(outlierCloudKeyFrames[thisKeyInd]));
                }
            }

            for (int i = 0; i < surroundingExistingKeyPosesID.size(); ++i)
            {
                *laserCloudCornerFromMap += *surroundingCornerCloudKeyFrames[i];
                *laserCloudSurfFromMap += *surroundingSurfCloudKeyFrames[i];
                *laserCloudSurfFromMap += *surroundingOutlierCloudKeyFrames[i];
            }
        }
        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->points.size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->points.size();
    }

    void downsampleCurrentScan()
    {

        laserCloudRawDS->clear();
        downSizeFilterScancontext.setInputCloud(laserCloudRaw);
        downSizeFilterScancontext.filter(*laserCloudRawDS);

        laserCloudCornerLastDS->clear();
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->points.size();
        // cout << "laserCloudCornerLastDSNum: " << laserCloudCornerLastDSNum << endl;

        laserCloudSurfLastDS->clear();
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->points.size();
        // cout << "laserCloudSurfLastDSNum: " << laserCloudSurfLastDSNum << endl;

        laserCloudOutlierLastDS->clear();
        downSizeFilterOutlier.setInputCloud(laserCloudOutlierLast);
        downSizeFilterOutlier.filter(*laserCloudOutlierLastDS);
        laserCloudOutlierLastDSNum = laserCloudOutlierLastDS->points.size();

        laserCloudSurfTotalLast->clear();
        laserCloudSurfTotalLastDS->clear();
        *laserCloudSurfTotalLast += *laserCloudSurfLastDS;
        *laserCloudSurfTotalLast += *laserCloudOutlierLastDS;
        downSizeFilterSurf.setInputCloud(laserCloudSurfTotalLast);
        downSizeFilterSurf.filter(*laserCloudSurfTotalLastDS);
        laserCloudSurfTotalLastDSNum = laserCloudSurfTotalLastDS->points.size();
    }

    void cornerOptimization(int iterCount)
    {

        updatePointAssociateToMapSinCos();
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0)
            {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++)
                {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;

                cv::eigen(matA1, matD1, matV1);

                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);

                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;

                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;

                    if (s > 0.1)
                    {
                        laserCloudOri->push_back(pointOri);
                        coeffSel->push_back(coeff);
                    }
                }
            }
        }
    }

    void surfOptimization(int iterCount)
    {
        updatePointAssociateToMapSinCos();
        for (int i = 0; i < laserCloudSurfTotalLastDSNum; i++)
        {
            pointOri = laserCloudSurfTotalLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0.at<float>(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0.at<float>(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0.at<float>(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                cv::solve(matA0, matB0, matX0, cv::DECOMP_QR);

                float pa = matX0.at<float>(0, 0);
                float pb = matX0.at<float>(1, 0);
                float pc = matX0.at<float>(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }

                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;

                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1)
                    {
                        laserCloudOri->push_back(pointOri);
                        coeffSel->push_back(coeff);
                    }
                }
            }
        }
    }

    bool LMOptimization(int iterCount)
    {
        float srx = sin(transformTobeMapped[0]);
        float crx = cos(transformTobeMapped[0]);
        float sry = sin(transformTobeMapped[1]);
        float cry = cos(transformTobeMapped[1]);
        float srz = sin(transformTobeMapped[2]);
        float crz = cos(transformTobeMapped[2]);

        int laserCloudSelNum = laserCloudOri->points.size();
        if (laserCloudSelNum < 50)
        {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));
        for (int i = 0; i < laserCloudSelNum; i++)
        {
            pointOri = laserCloudOri->points[i];
            coeff = coeffSel->points[i];

            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;

            matA.at<float>(i, 0) = arx;
            matA.at<float>(i, 1) = ary;
            matA.at<float>(i, 2) = arz;
            matA.at<float>(i, 3) = coeff.x;
            matA.at<float>(i, 4) = coeff.y;
            matA.at<float>(i, 5) = coeff.z;
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0)
        {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));

            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--)
            {
                if (matE.at<float>(0, i) < eignThre[i])
                {
                    for (int j = 0; j < 6; j++)
                    {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                }
                else
                {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }

        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }

        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);

        float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));

        if (deltaR < 0.05 && deltaT < 0.05)
        {
            return true;
        }
        return false;
    }

    void scan2MapOptimization()
    {

        if (laserCloudCornerFromMapDSNum > 10 && laserCloudSurfFromMapDSNum > 100)
        {

            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);

            for (int iterCount = 0; iterCount < 10; iterCount++)
            {

                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization(iterCount);
                surfOptimization(iterCount);

                if (LMOptimization(iterCount) == true)
                    break;
            }

            transformUpdate();
        }
    }

    void saveKeyFramesAndFactor()
    {

        currentRobotPosPoint.x = transformAftMapped[3];
        currentRobotPosPoint.y = transformAftMapped[4];
        currentRobotPosPoint.z = transformAftMapped[5];

        bool saveThisKeyFrame = true;
        if (sqrt((previousRobotPosPoint.x - currentRobotPosPoint.x) * (previousRobotPosPoint.x - currentRobotPosPoint.x) + (previousRobotPosPoint.y - currentRobotPosPoint.y) * (previousRobotPosPoint.y - currentRobotPosPoint.y) + (previousRobotPosPoint.z - currentRobotPosPoint.z) * (previousRobotPosPoint.z - currentRobotPosPoint.z)) < 0.3)
        { // save keyframe every 0.3 meter
            saveThisKeyFrame = false;
        }

        if (saveThisKeyFrame == false && !cloudKeyPoses3D->points.empty())
            return;

        previousRobotPosPoint = currentRobotPosPoint;

        /**
         * update grsam graph
         */
        if (cloudKeyPoses3D->points.empty())
        {
            gtSAMgraph.add(PriorFactor<Pose3>(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]), Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])), priorNoise));
            initialEstimate.insert(0, Pose3(Rot3::RzRyRx(transformTobeMapped[2], transformTobeMapped[0], transformTobeMapped[1]),
                                            Point3(transformTobeMapped[5], transformTobeMapped[3], transformTobeMapped[4])));
            for (int i = 0; i < 6; ++i)
                transformLast[i] = transformTobeMapped[i];
        }
        else
        {
            gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(transformLast[2], transformLast[0], transformLast[1]),
                                          Point3(transformLast[5], transformLast[3], transformLast[4]));
            gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                        Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4]));
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->points.size() - 1, cloudKeyPoses3D->points.size(), poseFrom.between(poseTo), odometryNoise));
            initialEstimate.insert(cloudKeyPoses3D->points.size(), Pose3(Rot3::RzRyRx(transformAftMapped[2], transformAftMapped[0], transformAftMapped[1]),
                                                                         Point3(transformAftMapped[5], transformAftMapped[3], transformAftMapped[4])));
        }
        /**
         * update iSAM
         */
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();

        gtSAMgraph.resize(0);
        initialEstimate.clear();

        /**
         * save key poses
         */
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;

        isamCurrentEstimate = isam->calculateEstimate();
        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);

        thisPose3D.x = latestEstimate.translation().y();
        thisPose3D.y = latestEstimate.translation().z();
        thisPose3D.z = latestEstimate.translation().x();
        thisPose3D.intensity = cloudKeyPoses3D->points.size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        cout << "cloudKeyPoses3D->points.size(): " << cloudKeyPoses3D->points.size() << endl;

        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().pitch();
        thisPose6D.pitch = latestEstimate.rotation().yaw();
        thisPose6D.yaw = latestEstimate.rotation().roll(); // in camera frame
        thisPose6D.time = timeLaserOdometry;
        cloudKeyPoses6D->push_back(thisPose6D);

        /**
         * save updated transform
         */
        if (cloudKeyPoses3D->points.size() > 1)
        {
            transformAftMapped[0] = latestEstimate.rotation().pitch();
            transformAftMapped[1] = latestEstimate.rotation().yaw();
            transformAftMapped[2] = latestEstimate.rotation().roll();
            transformAftMapped[3] = latestEstimate.translation().y();
            transformAftMapped[4] = latestEstimate.translation().z();
            transformAftMapped[5] = latestEstimate.translation().x();

            for (int i = 0; i < 6; ++i)
            {
                transformLast[i] = transformAftMapped[i];
                transformTobeMapped[i] = transformAftMapped[i];
            }
        }

        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisOutlierKeyFrame(new pcl::PointCloud<PointType>());

        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);
        pcl::copyPointCloud(*laserCloudOutlierLastDS, *thisOutlierKeyFrame);

        bool usingRawCloud = true;
        if (usingRawCloud)
        { // v2 uses downsampled raw point cloud, more fruitful height information than using feature points (v1)
            pcl::PointCloud<PointType>::Ptr thisRawCloudKeyFrame(new pcl::PointCloud<PointType>());
            pcl::copyPointCloud(*laserCloudRawDS, *thisRawCloudKeyFrame);
            scManager.makeAndSaveScancontextAndKeys(*thisRawCloudKeyFrame);
            // std::string rawDS_path = SceneFolder;
            // rawDS_path += to_string(laserCloudRawTime) + "_";
            // rawDS_path += to_string(cloudKeyPoses3D->points.size()) + "th_keyframe.pcd";
            // pcl::io::savePCDFileASCII(rawDS_path, *laserCloudRawDS);
        }
        else
        { // v1 uses thisSurfKeyFrame, it also works. (empirically checked at Mulran dataset sequences)
            scManager.makeAndSaveScancontextAndKeys(*thisSurfKeyFrame);
        }

        cornerCloudKeyFrames.push_back(thisCornerKeyFrame);
        surfCloudKeyFrames.push_back(thisSurfKeyFrame);
        outlierCloudKeyFrames.push_back(thisOutlierKeyFrame);
    } // saveKeyFramesAndFactor

    void correctPoses()
    {
        if (aLoopIsClosed == true)
        {
            recentCornerCloudKeyFrames.clear();
            recentSurfCloudKeyFrames.clear();
            recentOutlierCloudKeyFrames.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().z();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().x();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
            }

            aLoopIsClosed = false;
        }
    }

    void clearCloud()
    {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        laserCloudCornerFromMapDS->clear();
        laserCloudSurfFromMapDS->clear();
    }

    void run()
    {

        if (newLaserCloudCornerLast && std::abs(timeLaserCloudCornerLast - timeLaserOdometry) < 0.005 &&
            newLaserCloudSurfLast && std::abs(timeLaserCloudSurfLast - timeLaserOdometry) < 0.005 &&
            newLaserCloudOutlierLast && std::abs(timeLaserCloudOutlierLast - timeLaserOdometry) < 0.005 &&
            newLaserOdometry)
        {

            newLaserCloudCornerLast = false;
            newLaserCloudSurfLast = false;
            newLaserCloudOutlierLast = false;
            newLaserOdometry = false;

            std::lock_guard<std::mutex> lock(mtx);

            if (timeLaserOdometry - timeLastProcessing >= mappingProcessInterval)
            {

                timeLastProcessing = timeLaserOdometry;

                transformAssociateToMap();

                extractSurroundingKeyFrames();

                downsampleCurrentScan();

                scan2MapOptimization();

                saveKeyFramesAndFactor();

                correctPoses();

                publishTF();

                publishKeyPosesAndFrames();

                clearCloud();
            }
        }
    }
};

int main(int argc, char **argv)
{
    cout.setf(std::ios::fixed, std::ios::floatfield);
    ros::init(argc, argv, "lego_loam");

    ROS_INFO("\033[1;32m---->\033[0m Map Optimization Started.");

    mapOptimization MO;
    ros::NodeHandle nh_getparam;

    double Threshold;
    std::string gpsFailPath;
    std::string descriptor;

    nh_getparam.getParam("LocalizeFlag", MO.LocalizeFlag);
    nh_getparam.getParam("generateMap", MO.generateMap);
    nh_getparam.getParam("Scene", MO.Scene);
    nh_getparam.getParam("MapFrameNum", MO.MapFrameNum);
    nh_getparam.getParam("relocalHz", MO.relocalHz);
    nh_getparam.getParam("gpsDistThreshold", MO.distThreshold);
    nh_getparam.getParam("save_startind", MO.save_startind);
    nh_getparam.getParam("evalPath", MO.evalPath);
    nh_getparam.getParam("mapPospath", MO.mapPospath);
    nh_getparam.getParam("gtpath", MO.gtpath);

    nh_getparam.getParam("Threshold", Threshold);
    nh_getparam.getParam("gpsFailPath", gpsFailPath);
    nh_getparam.getParam("descriptor", descriptor);

    MO.SceneFolder = getenv("HOME");
    MO.SceneFolder += "/catkin_ws/data/pre_map/" + MO.Scene + "/keyframe/";
    MO.scManager.setThres(Threshold);
    MO.scManager.setgpsFailPath(gpsFailPath);
    MO.scManager.setDescriptor(descriptor);
    // MO.getRelocalParam(initLocalizeFlag, initgenerateMap, initScene, initMapFrameNum, Threshold, Hz, getgpsPath, getgpsFailPath, getgpsDistThreshold, getDescriptor);

    ros::Publisher pubMarker = nh_getparam.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    visualization_msgs::Marker line_list1, line_list2, line_list3, line_list4, line_list5;
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);
    if (MO.LocalizeFlag)
    {
        if (MO.generateMap)
        {
            ros::Rate rate(200);
            while (ros::ok())
            {
                ros::spinOnce();
                rate.sleep();
            }
            return 0;
        }
        std::thread RelocalThread(&mapOptimization::RelocalThread, &MO);

        float circle_angle = 0;
        int sector_index = 0, ring_index = 1;
        ros::Rate rate(200);
        MO.evaluationTxt.open(MO.evalPath, std::ios::app);
        while (ros::ok())
        // while ( 1 )
        {

            line_list1.header.frame_id = line_list2.header.frame_id = line_list3.header.frame_id = line_list4.header.frame_id = line_list5.header.frame_id = "/velodyne";
            line_list1.header.stamp = line_list2.header.stamp = line_list3.header.stamp = line_list4.header.stamp = line_list5.header.stamp = ros::Time::now();
            line_list1.ns = line_list2.ns = line_list3.ns = line_list4.ns = line_list5.ns = "lego_loam";
            line_list1.action = line_list2.action = line_list3.action = line_list4.action = line_list5.action = visualization_msgs::Marker::ADD;
            line_list1.pose.orientation.w = line_list2.pose.orientation.w = line_list3.pose.orientation.w = line_list4.pose.orientation.w = line_list5.pose.orientation.w = 1.0;
            line_list1.id = 0;
            line_list2.id = 1;
            line_list3.id = 2;
            line_list4.id = 3;
            line_list5.id = 4;
            line_list1.type = line_list2.type = line_list3.type = line_list4.type = line_list5.type = visualization_msgs::Marker::LINE_LIST;

            line_list1.scale.x = 0.02;
            line_list2.scale.x = 0.02;
            line_list3.scale.x = 0.1;
            line_list4.scale.x = 0.1;
            line_list5.scale.x = 0.1;

            line_list1.color.g = 1.0f;
            line_list1.color.a = 1.0;
            line_list2.color.g = 1.0f;
            line_list2.color.a = 1.0;
            line_list3.color.g = 1.0f;
            line_list3.color.a = 1.0;
            line_list4.color.g = 1.0f;
            line_list4.color.a = 1.0;
            line_list5.color.b = 1.0;
            line_list5.color.a = 1.0;

            geometry_msgs::Point p1;
            geometry_msgs::Point p2;
            geometry_msgs::Point p3;

            if (sector_index == 0)
            {
                p3.x = 0;
                p3.y = 0;
                p3.z = 0;
                line_list5.points.push_back(p3);
                p3.x = 80;
                line_list5.points.push_back(p3);
            }

            if (sector_index != 0 && sector_index != 60)
            {
                p1.x = 0;
                p1.y = 0;
                p1.z = 0;
                if (sector_index % 10 != 0)
                    line_list1.points.push_back(p1);
                else
                    line_list3.points.push_back(p1);
                p1.x = 80.0 * cos(sector_index * 6.0 / 180.0 * M_PI);
                p1.y = 80.0 * sin(sector_index * 6.0 / 180.0 * M_PI);
                if (sector_index % 10 != 0)
                    line_list1.points.push_back(p1);
                else
                    line_list3.points.push_back(p1);
            }

            if (ring_index != 21)
            {
                p2.x = ring_index * 4.0 * cos(circle_angle);
                p2.y = ring_index * 4.0 * sin(circle_angle);
                p2.z = 0;
                if (ring_index % 5 != 0)
                    line_list2.points.push_back(p2);
                else
                    line_list4.points.push_back(p2);
                p2.x = ring_index * 4.0 * cos(circle_angle + 6.0 / 180.0 * M_PI);
                p2.y = ring_index * 4.0 * sin(circle_angle + 6.0 / 180.0 * M_PI);
                if (ring_index % 5 != 0)
                    line_list2.points.push_back(p2);
                else
                    line_list4.points.push_back(p2);
            }

            pubMarker.publish(line_list1);
            pubMarker.publish(line_list2);
            pubMarker.publish(line_list3);
            pubMarker.publish(line_list4);
            pubMarker.publish(line_list5);

            ros::spinOnce(); // check for incoming messages
            // MO.run();
            rate.sleep();

            if (sector_index < 60)
                sector_index += 1;
            if (circle_angle < 2 * M_PI)
                circle_angle += 6.0 / 180.0 * M_PI;
            else
            {
                circle_angle = 0;
                if (ring_index < 21)
                    ring_index += 1;
            }
        }

        MO.evaluationTxt.close();
        RelocalThread.join();
        visualizeMapThread.join();
    }
    else
    {
        std::thread loopthread(&mapOptimization::loopClosureThread, &MO);

        ros::Rate rate(200);
        while (ros::ok())
        // while ( 1 )
        {
            ros::spinOnce();
            MO.run();
            rate.sleep();
        }

        visualizeMapThread.join();
        loopthread.join();
    }

    return 0;
}
