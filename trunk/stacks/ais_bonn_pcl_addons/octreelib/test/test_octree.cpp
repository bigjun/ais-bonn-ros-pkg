#include <ros/ros.h>

#include <octreelib/spatialaggregate/octree.h>

#include <stdlib.h>
#include <time.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <terminal_tools/time.h>

#include "pcl/visualization/pcl_visualizer.h"

#include <boost/thread/thread.hpp>



ros::Publisher pub;


using namespace spatialaggregate;


// to get a non-ambiguous constructor for int
template < typename ValueType, int Size >
class Vec : public Eigen::Matrix< ValueType, Size, 1 > {
public:
	Vec() {}
	Vec( const int v ) : Eigen::Matrix< ValueType, Size, 1 >( (ValueType) v ) {}
	~Vec() {}
};


void testOnRandomPoints() {

	terminal_tools::TicToc tictoc_;

	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
	viewer->setBackgroundColor (0, 0, 0);
	viewer->addCoordinateSystem (1.0);
	viewer->initCameraParameters ();

	{
		
		Eigen::Matrix< float, 4, 1 > center;
		center[0] = 0;
		center[1] = 0;
		center[2] = 0;
		center[3] = 1;
		
		const unsigned int numPoints = 1080*1000;
		boost::shared_ptr< OcTreeNodeFixedCountAllocator< float, float > > fixedCountAllocator = boost::make_shared< OcTreeNodeFixedCountAllocator< float, float > >( numPoints );
		
		for( unsigned int j = 0; j < 6; j++ )
		{
			
			boost::shared_ptr< OcTreeNodeAllocator< float, float > > allocator = boost::make_shared< OcTreeNodeAllocator< float, float > >();
			if( j >= 3 ) {
				allocator = fixedCountAllocator;
				allocator->reset();
			}
			
			OcTree< float, float > tree( center, 0.005f, 10.f, allocator );
			
			pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
			for( unsigned int i = 0; i < numPoints; i++ ) {

				pcl::PointXYZ p;

				p.x = rand() / ((float)RAND_MAX) * 10.f - 5.f;
				p.y = rand() / ((float)RAND_MAX) * 10.f - 5.f;
				p.z = rand() / ((float)RAND_MAX) * 1.f - 0.5f;
				
				cloud_ptr->points.push_back( p );
			}
			
			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> sample_color(cloud_ptr, 255, 0, 0);
//			viewer->addPointCloud<pcl::PointXYZ> (cloud_ptr, sample_color, "sample cloud");
//			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
			
			if( j >= 3 )
				std::cerr << "fixed count allocator";
			std::cerr << "adding " << numPoints << " random points\n";
			tictoc_.tic();
			for( unsigned int i = 0; i < numPoints; i++ ) {
				tree.addPoint( cloud_ptr->points[i].getArray4fMap(), 1.f, tree.max_depth_ );
			}
			double delta_t = tictoc_.toc() * 1000.0f;
	        std::cerr << "took " << delta_t << "ms.\n";

			Eigen::Matrix< float, 4, 1 > minPos, maxPos;
			
			minPos[0] = -1;
			minPos[1] = -1;
			minPos[2] = -1;
			
			maxPos[0] = 1;
			maxPos[1] = 1;
			maxPos[2] = 1;
			
			std::cerr << "querying leaves in range " << minPos << " " << maxPos << "\n";
			tictoc_.tic();
			std::list< OcTreeNode< float, float >* > nodes;
			tree.root_->getAllLeavesInVolume( nodes, tree.getKey(minPos), tree.getKey(maxPos), tree.max_depth_ );
//			tree.root_->getAllLeaves( nodes );
			delta_t = tictoc_.toc() * 1000.0f;
	        std::cerr << "took " << delta_t << "ms.\n";
			
			std::cerr << "found " << nodes.size() << " leaves, max_depth " << tree.max_depth_ << "\n";
			

			pcl::PointCloud<pcl::PointXYZ>::Ptr node_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
			for( std::list< OcTreeNode< float, float >* >::iterator it = nodes.begin(); it != nodes.end(); ++it ) {
				pcl::PointXYZ p;
				p.getArray4fMap() = (*it)->pos_key_.getPosition( &tree );
				node_cloud_ptr->points.push_back( p );
			}

			pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> node_color(cloud_ptr, 0, 255, 0);
			viewer->addPointCloud<pcl::PointXYZ> (node_cloud_ptr, node_color, "node cloud");
			viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "node cloud");


			std::cerr << "querying integral value in range " << minPos << " " << maxPos << "\n";
			tictoc_.tic();
			float value = tree.root_->getValueInVolume( tree.getKey(minPos), tree.getKey(maxPos), tree.max_depth_ );
			delta_t = tictoc_.toc() * 1000.0f;
	        std::cerr << "took " << delta_t << "ms.\n";
			
			std::cerr << "integral value " << value << "\n";
			
		}


		
	}
	
	
	while( !viewer->wasStopped() ) {
		viewer->spinOnce (100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}



}


void pointCloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg ) {

	pcl::PointCloud<pcl::PointXYZ> pointCloudIn;

	try
	{
		ROS_ERROR("[%s]: converting point cloud...", __PRETTY_FUNCTION__);
		pcl::fromROSMsg(*msg, pointCloudIn);
	}
	catch(std::exception &e)
	{
		ROS_ERROR("Exception while converting point cloud");
		return;
	}

	ROS_ERROR("[%s]: building tree with %i points...", __PRETTY_FUNCTION__, pointCloudIn.points.size());




	ROS_ERROR( "...done (not implemented)" );

}


int main(int argc, char** argv) {
	
	srand ( time(NULL) );

//	ros::init(argc, argv, "test_octree");
//	ros::NodeHandle n( "test_octree" );
	
	testOnRandomPoints();
	
//	// subscribe for point cloud data
//	ros::Subscriber sub = n.subscribe< sensor_msgs::PointCloud2 >( "input_cloud", 1, &pointCloudCallback );
//	pub = n.advertise< visualization_msgs::MarkerArray >( "visualization_marker_array", 10 );
//
//	ros::Rate loop_rate(100);
//	while( n.ok() ) {
//		ros::spinOnce();
//		loop_rate.sleep();
//	}
	

	
	return 0;
	
}




