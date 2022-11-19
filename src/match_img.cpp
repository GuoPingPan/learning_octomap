#include<iostream>
#include<vector>
#include<fstream>

#include<octomap/octomap.h>
#include<octomap/ColorOcTree.h>
#include<octomap/math/Pose6D.h>

#include<opencv2/core/core.hpp>
#include<opencv2/imgproc/imgproc.hpp>
#include<opencv2/highgui/highgui.hpp>

#include<eigen3/Eigen/Core>
#include<eigen3/Eigen/Geometry>

#include<pcl-1.8/pcl/common/transforms.h>
#include<pcl-1.8/pcl/point_types.h>


using namespace std;


float camera_scale = 1000.0;
float camera_cx = 325.5;
float camera_cy = 325.5;
float camera_fx = 325.5;
float camera_fy = 325.5;


int main(int argc, char **argv){

    ifstream in("../data/keyframe.txt");

    if(!in.is_open()){
        cout << "File don't exist." << endl;
        return 1;
    }

    vector<std::string> img_paths;

    while (in.peek() != EOF)
    {
        string s;
        in >> s;
        img_paths.push_back(s);
    }
    
    in.close();

    vector<Eigen::Isometry3d> transforms;

    in.open("../data/trajectory.txt");
    
    while (in.peek() != EOF)
    {
        double seq,x,y,z,qw,qx,qy,qz;
        in >> seq >> x >> y >> z >> qx >> qy >> qz >> qw;
        Eigen::Isometry3d q(Eigen::Quaterniond(qw,qx,qy,qz));
        
        q(0,3) = x; q(1,3) = y; q(2,3) = z;

        transforms.push_back(q);

    }
        
    in.close();

    octomap::ColorOcTree tree(0.05);

    for (size_t i = 0; i < img_paths.size(); i++)
    {
        cv::Mat color_img = cv::imread("../data/rgb_index/"+img_paths[i]+".ppm",-1);
        cv::Mat depth_img = cv::imread("../data/dep_index/"+img_paths[i]+".pgm",-1);

        pcl::PointCloud<pcl::PointXYZRGBA> cloud; 

        Eigen::Isometry3d& pose = transforms[i];

        for (int m = 0; m < color_img.rows; m++)
        {
            /* code */
            for (int n = 0; n < color_img.cols; n++)
            {
                /* code */
                ushort d = depth_img.ptr<ushort>(m)[n];
                if(d == 0) continue;
                float z = float(d) / camera_scale;
                float x = (n - camera_cx) * z / camera_fx;
                float y = (m - camera_cy) * z / camera_fy;

                pcl::PointXYZRGBA p;
                p.x = x; p.y = y; p.z = z;
                uchar *rgbdata = &color_img.ptr<uchar>(m)[n*3];

                p.b = rgbdata[0]; p.g = rgbdata[1]; p.r = rgbdata[2];
                
                cloud.points.push_back(p);
            }
            
        }
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr temp(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::transformPointCloud(cloud, *temp, pose.matrix());

        octomap::Pointcloud cloud_octo;
        for(auto p:temp->points)
            cloud_octo.push_back(p.x, p.y, p.z);
        
        tree.insertPointCloud( cloud_octo, octomap::point3d( pose(0,3), pose(1,3), pose(2,3)) );

        for(auto p:temp->points)
            tree.integrateNodeColor( p.x, p.y, p.z, p.r, p.g, p.b );

    }
    


    tree.updateInnerOccupancy();
    tree.write( "../data/map.ot" );
 
    cout<<"done."<<endl;


    return 0;
}