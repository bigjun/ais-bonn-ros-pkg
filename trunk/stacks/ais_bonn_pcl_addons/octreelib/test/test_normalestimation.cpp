#include <ros/ros.h>

#include <octreelib/spatialaggregate/octree.h>
#include <octreelib/feature/normalestimation.h>

#include <stdlib.h>
#include <time.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/publisher.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


ros::Publisher pub;
pcl_ros::Publisher< pcl::PointXYZRGB > pub_cloud;


using namespace spatialaggregate;
using namespace algorithm;
using namespace feature;

boost::shared_ptr< OcTreeNodeFixedCountAllocator< float, NormalEstimationValue > > fixedCountAllocator;

void testOnRandomPoints() {

	const int numPoints = 640*480;
	const float maxRange = 10.f;
	const float minResolution = 0.005f;
	
	fixedCountAllocator = boost::make_shared< OcTreeNodeFixedCountAllocator< float, NormalEstimationValue > >( numPoints );
	for( unsigned int j = 0; j < 10; j++ )
	{
		
		pcl::PointCloud< pcl::PointXYZRGB > points;
		points.header.frame_id = "openni_camera";
		points.header.stamp = ros::Time::now();
		points.width = numPoints;
		points.height = 1;
		points.is_dense = false;
		points.points.resize( numPoints );
		
		std::vector< int > indices( numPoints );
		
		for( unsigned int i = 0; i < numPoints; i++ ) {
			
			pcl::PointXYZRGB& p = points.points[i];
			
			p.x = rand() / ((float)RAND_MAX) * 10.f - 5.f;
			p.y = rand() / ((float)RAND_MAX) * 10.f - 5.f;
			p.z = rand() / ((float)RAND_MAX) * 0.1f - 0.05f + 2.f;
			p.rgb = 0.f;
			
			indices[i] = i;
			
		}
		
		
		fixedCountAllocator->reset();
		
		int octreeDepth = 0;
		OcTreeSamplingMap< float, NormalEstimationValue > octreeSamplingMap;

		ROS_ERROR("calculating normals on %i points ", numPoints);
		ros::Time startTime = ros::Time::now();
		boost::shared_ptr< spatialaggregate::OcTree<float, NormalEstimationValue> > octree = buildNormalEstimationOctree< float, NormalEstimationValue, pcl::PointXYZRGB >( points, indices, octreeDepth, octreeSamplingMap, maxRange, minResolution, fixedCountAllocator );
		for( int i = 0; i <= octreeDepth; i++ )
			calculateNormalsOnOctreeLayer< float, NormalEstimationValue >( octreeSamplingMap[i], 25 );
		ros::Time stopTime = ros::Time::now();
		ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
		
		
		ROS_ERROR( "visualization..." );
		
		if( true )
			pub_cloud.publish( points );
		
		if( true ) {
			
			visualization_msgs::MarkerArray markerArray;
			
			for( int i = 0; i <= octreeDepth; i++ ) {
				
				visualization_msgs::Marker marker;
				marker.header.frame_id = "openni_camera";
				marker.header.stamp = ros::Time::now();
				char str[255];
				sprintf( str, "normals %i", i );
				marker.ns = str;
				marker.id = 1;
				marker.type = visualization_msgs::Marker::LINE_LIST;
				marker.action = visualization_msgs::Marker::ADD;
				marker.points.clear();
				marker.scale.x = 0.005f * (float)(octreeDepth-i+1);
				marker.scale.y = 0.1f;
				marker.scale.z = 0.f;

				marker.color.r = 1.f;
				marker.color.g = 0.f;
				marker.color.b = 0.f;
				marker.color.a = 1.f;
				
				marker.lifetime = ros::Duration(0.0);
				
				for( int j = 0; j < octreeSamplingMap[i].size(); j++ ) {
					
					OcTreeNode< float, NormalEstimationValue >* n = octreeSamplingMap[i][j];
					
					if( !n->value.stable )
						continue;
					
					float invCount = 1.f / ((float)n->numPoints);
					float nodeWidth = n->maxPosition[0] - n->minPosition[0];
					
					geometry_msgs::Point p1, p2;
					p1.x = n->value.summedPos[0] * invCount;
					p1.y = n->value.summedPos[1] * invCount;
					p1.z = n->value.summedPos[2] * invCount;
					p2.x = n->value.summedPos[0] * invCount + n->value.normal[0] * nodeWidth;
					p2.y = n->value.summedPos[1] * invCount + n->value.normal[1] * nodeWidth;
					p2.z = n->value.summedPos[2] * invCount + n->value.normal[2] * nodeWidth;
					
					marker.points.push_back( p1 );
					marker.points.push_back( p2 );
					
					std_msgs::ColorRGBA c;
					c.r = 0.5f + 0.5f * n->value.normal[0];
					c.g = 0.5f + 0.5f * n->value.normal[1];
					c.b = 0.5f + 0.5f * n->value.normal[2];
					marker.colors.push_back( c );
					marker.colors.push_back( c );
				}
				
				markerArray.markers.push_back( marker );
			}
			
			pub.publish( markerArray );
		}
		
		
	}

}


void pointCloudCallback( const sensor_msgs::PointCloud2ConstPtr& msg ) {
	
	pcl::PointCloud<pcl::PointXYZRGB> pointCloudIn;
	
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

	const unsigned int numPoints = pointCloudIn.points.size();
	const float maxRange = 30.f;
	const float minResolution = 0.005f;
	
	std::vector< int > indices( numPoints );
	for( unsigned int i = 0; i < numPoints; i++ )
		indices[i] = i;
	
	fixedCountAllocator = boost::make_shared< OcTreeNodeFixedCountAllocator< float, NormalEstimationValue > >( numPoints );
	
	for( int j = 0; j < 2; j++ ) {
		
		fixedCountAllocator->reset();
		
		int octreeDepth = 0;
		OcTreeSamplingMap< float, NormalEstimationValue > octreeSamplingMap;

		ROS_ERROR("calculating normals on %i points ", numPoints);
		ros::Time startTime = ros::Time::now();
		boost::shared_ptr< spatialaggregate::OcTree<float, NormalEstimationValue> > octree = buildNormalEstimationOctree< float, NormalEstimationValue, pcl::PointXYZRGB >( pointCloudIn, indices, octreeDepth, octreeSamplingMap, maxRange, minResolution, fixedCountAllocator );
		for( int i = 0; i <= octreeDepth; i++ )
			calculateNormalsOnOctreeLayer< float, NormalEstimationValue >( octreeSamplingMap[i], 10 );
		ros::Time stopTime = ros::Time::now();
		ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
		
		
		ROS_ERROR( "visualization..." );
		
/*		if( j == 0 )
			pub_cloud.publish( pointCloudIn );*/
		
		if( j == 0 ) {
			
			visualization_msgs::MarkerArray markerArray;
			
			for( int i = 0; i <= octreeDepth; i++ ) {
				
				visualization_msgs::Marker marker;
				marker.header = msg->header;
				char str[255];
				sprintf( str, "normals %i", i );
				marker.ns = str;
				marker.id = 1;
				marker.type = visualization_msgs::Marker::LINE_LIST;
				marker.action = visualization_msgs::Marker::ADD;
				marker.points.clear();
				marker.scale.x = 0.002 * (octreeDepth-i+1);
				marker.scale.y = 0.002f * (octreeDepth-i+1);
				marker.scale.z = 0.f;

				marker.color.r = 1.f;
				marker.color.g = 0.f;
				marker.color.b = 0.f;
				marker.color.a = 1.f;
				
				marker.lifetime = ros::Duration(0.0);
				
				for( int j = 0; j < octreeSamplingMap[i].size(); j++ ) {
					
					OcTreeNode< float, NormalEstimationValue >* n = octreeSamplingMap[i][j];
					
					if( !n->value.stable )
						continue;
					
					float invCount = 1.f / ((float)n->numPoints);
					float nodeWidth = n->maxPosition[0] - n->minPosition[0];
					
					geometry_msgs::Point p1, p2;
					p1.x = n->value.summedPos[0] * invCount;
					p1.y = n->value.summedPos[1] * invCount;
					p1.z = n->value.summedPos[2] * invCount;
					p2.x = n->value.summedPos[0] * invCount + 0.05f*(octreeDepth-i+1)*n->value.normal[0];
					p2.y = n->value.summedPos[1] * invCount + 0.05f*(octreeDepth-i+1)*n->value.normal[1];
					p2.z = n->value.summedPos[2] * invCount + 0.05f*(octreeDepth-i+1)*n->value.normal[2];
					
					marker.points.push_back( p1 );
					marker.points.push_back( p2 );
					
					std_msgs::ColorRGBA c;
					c.r = 0.5f + 0.5f * n->value.normal[0];
					c.g = 0.5f + 0.5f * n->value.normal[1];
					c.b = 0.5f + 0.5f * n->value.normal[2];
					marker.colors.push_back( c );
					marker.colors.push_back( c );
					
				}
				
				markerArray.markers.push_back( marker );
			}
			
			pub.publish( markerArray );
			
		}
		
	}

}


int main(int argc, char** argv) {
	
	srand ( time(NULL) );

	ros::init(argc, argv, "test_octree_normalestimation");
	ros::NodeHandle n( "test_octree_normalestimation" );
	
	
	// subscribe for point cloud data
	ros::Subscriber sub = n.subscribe< sensor_msgs::PointCloud2 >( "input_cloud", 1, &pointCloudCallback );	
	pub = n.advertise< visualization_msgs::MarkerArray >( "visualization_marker_array", 10 );
	pub_cloud = pcl_ros::Publisher< pcl::PointXYZRGB >( n, "output_cloud", 1 );
	
	if( argc == 1 ) {
		ROS_ERROR("usage: no args: feed in point cloud data, arg random: test on random data");
	}
	
	if( argc > 1 && std::string( argv[1] ) == "random" )
		testOnRandomPoints();
	
	ros::Rate loop_rate(100);
	while( n.ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
	
}




