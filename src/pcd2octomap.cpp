#include<iostream>
#include<pcl-1.8/pcl/io/pcd_io.h>
#include<pcl-1.8/pcl/point_types.h>

#include<octomap/octomap.h>

using namespace std;

int main(int argc, char **argv){

    if(argc != 3){

        cout <<"Usage: pcd2octomap <input_file> <output_file>" << endl;
        return 1;
    }

    string input_file = argv[1], output_file = argv[2];

    pcl::PointCloud<pcl::PointXYZRGBA> cloud;
    pcl::io::loadPCDFile<pcl::PointXYZRGBA>(input_file, cloud);

    cout << "The PCD point size: " << cloud.size() << endl;

    octomap::OcTree tree(0.05);

    for(auto &p:cloud){
        tree.updateNode(octomap::point3d(p.x,p.y,p.z), true);
    }

    tree.updateInnerOccupancy();
    
    tree.writeBinary(output_file);

    cout << "done." << endl;

    return 0;
}