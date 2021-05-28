#include "iostream"
#include "Scancontext.h"
#include <pcl/io/pcd_io.h>

using namespace std;

SCManager scManager;

int main (int argc,char **argv)
{
    // string path1 = "/home/zeng/caculateSCI/1/1.pcd";
    // string path2 = "/home/zeng/caculateSCI/1/2.pcd";
    // pcl::PointCloud<pcl::PointXYZI>::Ptr loadRawCloud1(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::PointCloud<pcl::PointXYZI>::Ptr loadRawCloud2(new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::io::loadPCDFile(path1, *loadRawCloud1);
    // pcl::io::loadPCDFile(path2, *loadRawCloud2);
    // Eigen::MatrixXd sc1 = scManager.makeScancontext(*loadRawCloud1);
    // Eigen::MatrixXd sc2 = scManager.makeScancontext(*loadRawCloud2);
    // auto result = scManager.distanceBtnScanContext(sc1,sc2);
    // cout<<result.first<<endl;
    // cv::Mat sciForRedPoint = scManager.createSci(sc1);
    // Eigen::MatrixXd scShift = circshift(sc2,result.second);
    // cv::Mat sciForWhitePoint = scManager.createSci(scShift);
    // // cv::imshow("red point", sciForRedPoint);
    // // cv::imshow("white point", sciForWhitePoint);
    // // cv::waitKey(0);
    // cv::imwrite("/home/zeng/caculateSCI/1/1.png",sciForRedPoint);
    // cv::imwrite("/home/zeng/caculateSCI/1/2.png",sciForWhitePoint);
    
int num = 10;
int th[num] ={83,84,124,125,126,133,134,138,334,1241};
for(int jk = 0; jk < num; ++jk)
    {
        char path[100];
        sprintf(path,"/home/ubuwgb/caculateSCI/1/%d.pcd",th[jk]);
        pcl::PointCloud<pcl::PointXYZI>::Ptr loadRawCloud1(new pcl::PointCloud<pcl::PointXYZI>());
        pcl::io::loadPCDFile(path, *loadRawCloud1);
        Eigen::MatrixXd sc1 = scManager.makeScancontext(*loadRawCloud1);
        cv::Mat sci1 = scManager.createSci(sc1);
        char name[100];
        sprintf(name,"/home/ubuwgb/caculateSCI/1/%d.png",th[jk]);
        cv::imwrite(name,sci1);
    }
}