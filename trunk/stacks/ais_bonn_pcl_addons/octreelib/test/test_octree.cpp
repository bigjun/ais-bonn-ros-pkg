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

	{
	
		OcTreePosition< float > dimensions, center;
		dimensions[0] = 20;
		dimensions[1] = 20;
		dimensions[2] = 20;
		center[0] = 0;
		center[1] = 0;
		center[2] = 0;
		
		{
			OcTree< float, float > tree( dimensions, center );
			
			OcTreePoint< float, float > point;
			point.position[0] = 0;
			point.position[1] = 0;
			point.position[2] = 0;
			ROS_ERROR("adding point %f %f %f", point.position[0], point.position[1], point.position[2]);
			tree.addPoint( point );
			
			point.position[0] = 0;
			point.position[1] = 1;
			point.position[2] = 0;
			ROS_ERROR("adding point %f %f %f", point.position[0], point.position[1], point.position[2]);
			tree.addPoint( point );

			point.position[0] = 0;
			point.position[1] = 1;
			point.position[2] = 1;
			ROS_ERROR("adding point %f %f %f", point.position[0], point.position[1], point.position[2]);
			tree.addPoint( point );

			point.position[0] = -1;
			point.position[1] = 0;
			point.position[2] = 0;
			ROS_ERROR("adding point %f %f %f", point.position[0], point.position[1], point.position[2]);
			tree.addPoint( point );
			
			
			OcTreePosition< float > minPos, maxPos;
			
			minPos[0] = 0;
			minPos[1] = 0;
			minPos[2] = 0;
			
			maxPos[0] = 0.1;
			maxPos[1] = 0.1;
			maxPos[2] = 0.1;
			
			ROS_ERROR("querying points in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			std::vector< OcTreeNode< float, float >* > nodes = tree.getPointsInVolume( minPos, maxPos );
			
			for( unsigned int i = 0; i < nodes.size(); i++ ) {
				ROS_ERROR("found point %f %f %f", nodes[i]->position[0], nodes[i]->position[1], nodes[i]->position[2]);
			}
			
			
			minPos[0] = -1;
			minPos[1] = -1;
			minPos[2] = -1;
			
			maxPos[0] = 1.1;
			maxPos[1] = 1.1;
			maxPos[2] = 1.1;
			
			ROS_ERROR("querying points in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			nodes = tree.getPointsInVolume( minPos, maxPos );
			
			for( unsigned int i = 0; i < nodes.size(); i++ ) {
				ROS_ERROR("found point %f %f %f", nodes[i]->position[0], nodes[i]->position[1], nodes[i]->position[2]);
			}
			
			
			minPos[0] = 0;
			minPos[1] = 0;
			minPos[2] = 0;
			
			maxPos[0] = 0.01;
			maxPos[1] = 0.01;
			maxPos[2] = 0.01;
			
			ROS_ERROR("querying points in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			nodes = tree.getPointsInVolume( minPos, maxPos );
			
			for( unsigned int i = 0; i < nodes.size(); i++ ) {
				ROS_ERROR("found point %f %f %f", nodes[i]->position[0], nodes[i]->position[1], nodes[i]->position[2]);
			}
			
		}
		
	}
	
	{
		
		OcTreePosition< float > dimensions, center;
		dimensions[0] = 20;
		dimensions[1] = 20;
		dimensions[2] = 20;
		center[0] = 0;
		center[1] = 0;
		center[2] = 0;
		
		const unsigned int numPoints = 1080*1000;
		OcTreeNodeFixedCountAllocator< float, float >* fixedCountAllocator = new OcTreeNodeFixedCountAllocator< float, float >( numPoints );
		
		for( unsigned int j = 0; j < 6; j++ )
		{
			
			OcTreeNodeAllocator< float, float >* allocator = NULL;
			if( j >= 3 )
				allocator = fixedCountAllocator;
			
			if( allocator )
				allocator->reset();
			OcTree< float, float > tree( dimensions, center, 0, allocator );
			
			std::vector< OcTreePoint< float, float > > points;
			for( unsigned int i = 0; i < numPoints; i++ ) {
				OcTreePoint< float, float > point;
				point.value = 1;
				point.position[0] = rand() / ((float)RAND_MAX) * 10.f - 5.f;
				point.position[1] = rand() / ((float)RAND_MAX) * 10.f - 5.f;
				point.position[2] = rand() / ((float)RAND_MAX) * 10.f - 5.f;
				
				points.push_back( point );
			}
			
			
			if( j >= 3 )
				ROS_ERROR("fixed count allocator");
			ROS_ERROR("adding %i random points: ", numPoints);
			ros::Time startTime = ros::Time::now();
			for( unsigned int i = 0; i < numPoints; i++ ) {
				tree.addPoint( points[i] );
			}
			ros::Time stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );

			OcTreePosition< float > minPos, maxPos;
			
			minPos[0] = -1;
			minPos[1] = -1;
			minPos[2] = -1;
			
			maxPos[0] = 1;
			maxPos[1] = 1;
			maxPos[2] = 1;
			
			ROS_ERROR("querying points in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			startTime = ros::Time::now();
			std::vector< OcTreeNode< float, float >* > nodes = tree.getPointsInVolume( minPos, maxPos );
			stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			ROS_ERROR("found %i points", nodes.size());
			
	/*		for( unsigned int i = 0; i < nodes.size(); i++ ) {
				ROS_ERROR("found point %f %f %f", nodes[i]->position[0], nodes[i]->position[1], nodes[i]->position[2]);
			}*/


			ROS_ERROR("querying integral value in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			startTime = ros::Time::now();
			float value = tree.getValueInVolume( minPos, maxPos, 0 );
			stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			ROS_ERROR("integral value %f", value);
			
			
		}


		
	}
	
	
	{
	
		OcTreePosition< float > dimensions, center;
		dimensions[0] = 2000;
		dimensions[1] = 2000;
		dimensions[2] = 2000;
		center[0] = 0;
		center[1] = 0;
		center[2] = 0;
		
		for( unsigned int j = 0; j < 1; j++ )
		{
			OcTree< float, int > tree( dimensions, center, 0 );
			
			OcTreePosition< float > minPos, maxPos;
			
			minPos[0] = -100;
			minPos[1] = -100;
			minPos[2] = -100;
			
			maxPos[0] = 100;
			maxPos[1] = 100;
			maxPos[2] = 100;
			
			unsigned int pointsInSearchVolume = 0;
			
			const unsigned int numPoints = 1080*1000;
			ROS_ERROR("adding %i random points: ", numPoints);
			ros::Time startTime = ros::Time::now();
			for( unsigned int i = 0; i < numPoints; i++ ) {
				OcTreePoint< float, int > point;
				point.value = 1;
				point.position[0] = rand() / ((float)RAND_MAX) * 1000 - 500;
				point.position[1] = rand() / ((float)RAND_MAX) * 1000 - 500;
				point.position[2] = rand() / ((float)RAND_MAX) * 1000 - 500;
				
				if( point.position[0] >= minPos[0] && point.position[0] <= maxPos[0] &&
					point.position[1] >= minPos[1] && point.position[1] <= maxPos[1] &&
					point.position[2] >= minPos[2] && point.position[2] <= maxPos[2] )
					pointsInSearchVolume++;
				
				tree.addPoint( point );
				
			}
			ros::Time stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			
			ROS_ERROR("querying points in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			startTime = ros::Time::now();
			std::vector< OcTreeNode< float, int >* > nodes = tree.getPointsInVolume( minPos, maxPos );
			stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			ROS_ERROR("found %i of %i points", nodes.size(), pointsInSearchVolume);
			

			ROS_ERROR("querying integral value in range %f,%f,%f -> %f,%f,%f", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			startTime = ros::Time::now();
			int value = 0;
			unsigned int count = 0;
			tree.getValueAndCountInVolume( value, count, minPos, maxPos, 0 );
			stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			ROS_ERROR("integral value %i, count %i", value, count);
			
			
		}

	}
	
	
	{
	
		OcTreePosition< int > dimensions, center;
		dimensions[0] = 2000;
		dimensions[1] = 2000;
		dimensions[2] = 2000;
		center[0] = 0;
		center[1] = 0;
		center[2] = 0;
		
		for( unsigned int j = 0; j < 1; j++ )
		{
			OcTree< int, int > tree( dimensions, center, 0 );
			
			OcTreePosition< int > minPos, maxPos;
			
			minPos[0] = -100;
			minPos[1] = -100;
			minPos[2] = -100;
			
			maxPos[0] = 100;
			maxPos[1] = 100;
			maxPos[2] = 100;
			
			unsigned int pointsInSearchVolume = 0;
			
			const unsigned int numPoints = 1080*1000;
			ROS_ERROR("adding %i random points: ", numPoints);
			ros::Time startTime = ros::Time::now();
			for( unsigned int i = 0; i < numPoints; i++ ) {
				OcTreePoint< int, int > point;
				point.value = 1;
				point.position[0] = rand() / ((float)RAND_MAX) * 1000 - 500;
				point.position[1] = rand() / ((float)RAND_MAX) * 1000 - 500;
				point.position[2] = rand() / ((float)RAND_MAX) * 1000 - 500;
				
				if( point.position[0] >= minPos[0] && point.position[0] <= maxPos[0] &&
					point.position[1] >= minPos[1] && point.position[1] <= maxPos[1] &&
					point.position[2] >= minPos[2] && point.position[2] <= maxPos[2] )
					pointsInSearchVolume++;
				
				tree.addPoint( point );
				
			}
			ros::Time stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			
			ROS_ERROR("querying points in range %i,%i,%i -> %i,%i,%i", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			startTime = ros::Time::now();
			std::vector< OcTreeNode< int, int >* > nodes = tree.getPointsInVolume( minPos, maxPos );
			stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			ROS_ERROR("found %i of %i points", nodes.size(), pointsInSearchVolume);
			

			ROS_ERROR("querying integral value in range %i,%i,%i -> %i,%i,%i", minPos[0], minPos[1], minPos[2], maxPos[0], maxPos[1], maxPos[2]);
			startTime = ros::Time::now();
			int value = 0;
			unsigned int count = 0;
			tree.getValueAndCountInVolume( value, count, minPos, maxPos, 0 );
			stopTime = ros::Time::now();
			ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
			
			ROS_ERROR("integral value %i, count %i", value, count);
			
			
		}

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

	OcTreePosition< float > dimensions, center;
	dimensions[0] = 60;
	dimensions[1] = 60;
	dimensions[2] = 60;
	center[0] = 0;
	center[1] = 0;
	center[2] = 0;
		
	const float minTreeLeafSize = 0.005f;//1e-4f;
	OcTree< float, Vec< float, 4 > > tree( dimensions, center, minTreeLeafSize );
	
	std::vector< OcTreeNode< float, Vec< float, 4 > >* > pointNodes;
	
	std::vector< unsigned int > representativeNodeIndices;
	std::vector< unsigned int > representativeFirstDepths;
	
	float minVolumeSize = 60;
	float avgVolumeSize = 0;
	
	unsigned int maxDepth = 0;
	
	ros::Time startTime = ros::Time::now();
	for( unsigned int i = 0; i < pointCloudIn.points.size(); i++ ) {
		OcTreePoint< float, Vec< float, 4 > > point;
		point.position[0] = pointCloudIn.points[i].x;
		point.position[1] = pointCloudIn.points[i].y;
		point.position[2] = pointCloudIn.points[i].z;
		
		point.value[0] = point.position[0];
		point.value[1] = point.position[1];
		point.value[2] = point.position[2];
		point.value[3] = 1;
		
		OcTreeNode< float, Vec< float, 4 > >* n = tree.addPoint( point );
		pointNodes.push_back( n );
		
		if( n->numPoints <= 1 ) {
			representativeNodeIndices.push_back( i );
			representativeFirstDepths.push_back( n->depth );
		}
		
		if( n->depth > maxDepth )
			maxDepth = n->depth;
		
		float volumeSize = n->maxPosition[0] - n->minPosition[0];
		if( volumeSize < minVolumeSize )
			minVolumeSize = volumeSize;
		
		avgVolumeSize += volumeSize;
		
	}
	
	// build layer point clouds
	std::vector< std::vector< OcTreeNode< float, Vec< float, 4 > >* > > pointNodesLayered( maxDepth );
	for( unsigned int i = 0; i < representativeNodeIndices.size(); i++ ) {
		for( unsigned int l = representativeFirstDepths[i]-1; l < pointNodes[i]->depth; l++ ) {
			pointNodesLayered[l].push_back( pointNodes[i] );
		}
	}
	
	
	ros::Time stopTime = ros::Time::now();
	ROS_ERROR( "took: %f, min vol size %lf, avg %lf, max depth %i", (stopTime-startTime).toSec(), minVolumeSize, avgVolumeSize / ((float)pointCloudIn.points.size()), maxDepth );
	
	
	for( unsigned int l = 0; l < maxDepth; l++ ) {
		
		if( pointNodesLayered[l].size() > 0 ) {
			float volumeSize = 60.f * pow( 0.5f, (float)l + 1.f );
			ROS_ERROR( "num points at depth %i (leaf size %f): %i", l, volumeSize, pointNodesLayered[l].size() );
		}
		
	}
	
	std::vector< spatialaggregate::OcTreeNode< float, Vec< float, 4 > >* > nodesAll;
	std::vector< spatialaggregate::OcTreeNode< float, Vec< float, 4 > >* > nodesByDepth[ maxDepth ];
	tree.root->collectNodesInDepthRange( nodesAll, 1, maxDepth );
	for( unsigned int i = 0; i < nodesAll.size(); i++ ) {
		
		nodesByDepth[ nodesAll[i]->depth-1 ].push_back( nodesAll[i] );
		
/*		const float size = (nodesAll[i]->maxPosition[0] - nodesAll[i]->minPosition[0]);
		nodesAll[i]->value[4] = size * size * size / nodesAll[i]->value[4];*/
	}
	
	if( true )
	{
		// calc some average through the layers
		ROS_ERROR("querying integral value throughout layers to get error");
		for( unsigned int l = 0; l < maxDepth; l++ ) {
			
			float volumeSize = 60.f * pow( 0.5f, (float)l + 1.f );
			
			const float rangeSize = 2.f*2.f*volumeSize;
			const float minLeafSize = 0.5f*0.5f*volumeSize;
			OcTreeNode< float, Vec< float, 4 > >* tightestNode = NULL;
			
			float maxError = 0;
			float avgError = 0;
			for( unsigned int i = 0; i < pointNodesLayered[l].size(); i++ ) {
				OcTreePosition< float > minPos, maxPos;
				
				for( unsigned int j = 0; j < 3; j++ ) {
					minPos[j] = pointNodesLayered[l][i]->position[j] - rangeSize;
					maxPos[j] = pointNodesLayered[l][i]->position[j] + rangeSize;
				}
				
				if( i == 0 )
					tightestNode = pointNodesLayered[l][i]->getTightestNode( minPos, maxPos, minLeafSize );
				else
					tightestNode = tightestNode->getTightestNode( minPos, maxPos, minLeafSize );
				
				Vec< float, 4 > valueExact, valueApproximate;
				unsigned int countExact, countApproximate;
			
				tree.getValueAndCountInVolume( valueExact, countExact, minPos, maxPos, 0 );
				tightestNode->getValueAndCountInVolume( valueApproximate, countApproximate, minPos, maxPos, minLeafSize );
				
				if( countExact > 0 && countApproximate > 0 ) {
					valueExact /= countExact;
					valueApproximate /= countApproximate;
					
					avgError += (valueExact - valueApproximate).norm();
					maxError = std::max( maxError, (valueExact - valueApproximate).norm() );
				}
				
			}
			
			if( pointNodesLayered[l].size() > 0 )
				avgError /= pointNodesLayered[l].size();
			ROS_ERROR( "layer %i, leaf size %f, range %f, min leaf size %f, avg error %f, max error %f, rel error %f", l, volumeSize, rangeSize, minLeafSize, avgError, maxError, avgError / (2.f*rangeSize) );


		}

		// calc some average through the layers
		ROS_ERROR("querying integral value throughout layers");
		startTime = ros::Time::now();
		for( unsigned int l = 0; l < maxDepth; l++ ) {
			
			float volumeSize = 60.f * pow( 0.5f, (float)l + 1.f );
			
			const float rangeSize = 2.f*2.f*volumeSize;
			const float minLeafSize = 0.5f*0.5f*volumeSize;
			OcTreeNode< float, Vec< float, 4 > >* tightestNode = NULL;
			
			for( unsigned int i = 0; i < pointNodesLayered[l].size(); i++ ) {
				OcTreePosition< float > minPos, maxPos;
				
				for( int j = 0; j < 3; j++ ) {
					minPos[j] = pointNodesLayered[l][i]->position[j] - rangeSize;
					maxPos[j] = pointNodesLayered[l][i]->position[j] + rangeSize;
				}
				
				tightestNode = pointNodesLayered[l][i]->getTightestNode( minPos, maxPos, minLeafSize );
				
				Vec< float, 4 > valueApproximate;
				unsigned int countApproximate;
			
				tightestNode->getValueAndCountInVolume( valueApproximate, countApproximate, minPos, maxPos, minLeafSize );
				
			}

		}
		stopTime = ros::Time::now();
		ROS_ERROR( "took: %f", (stopTime-startTime).toSec() );
	}
	

	ROS_ERROR( "visualization..." );
	
	// visualize nodes with sufficient density
	if( false ) {
		
		const int vizDepth = maxDepth-4;
		
		visualization_msgs::MarkerArray markerArray;
		markerArray.markers.reserve( nodesByDepth[vizDepth].size() );
		
		for( unsigned int i = 0; i < nodesByDepth[vizDepth].size(); i++ ) {
			
			spatialaggregate::OcTreeNode< float, Vec< float, 4 > >* n = nodesByDepth[vizDepth][i];
			
			if( n->numPoints < 20 )
				continue;
			
			visualization_msgs::Marker marker;
			marker.header.frame_id = msg->header.frame_id;
			marker.header.stamp = ros::Time::now();
			marker.ns = "octree leaf points";
			marker.id = markerArray.markers.size();
			marker.type = visualization_msgs::Marker::CUBE;
			marker.action = visualization_msgs::Marker::ADD;
			
			marker.pose.position.x = 0.5f * (n->maxPosition[0] + n->minPosition[0]);
			marker.pose.position.y = 0.5f * (n->maxPosition[1] + n->minPosition[1]);
			marker.pose.position.z = 0.5f * (n->maxPosition[2] + n->minPosition[2]);
			
			marker.scale.x = (n->maxPosition[0] - n->minPosition[0]);
			marker.scale.y = (n->maxPosition[1] - n->minPosition[1]);
			marker.scale.z = (n->maxPosition[2] - n->minPosition[2]);

			marker.color.r = 1.f;
			marker.color.g = 0.f;
			marker.color.b = 0.f;
			marker.color.a = 0.2f;
			
			marker.lifetime = ros::Duration(0.0);
			
			markerArray.markers.push_back( marker );
			
		}
		
		pub.publish( markerArray );
	}
	

	ROS_ERROR( "...done" );

}


int main(int argc, char** argv) {
	
	srand ( time(NULL) );

	ros::init(argc, argv, "test_octree");
	ros::NodeHandle n( "test_octree" );
	
	if( argc == 1 ) {
		ROS_ERROR("usage: no args: feed in point cloud data, arg random: test on random data");
	}
	
	if( argc > 1 && std::string( argv[1] ) == "random" )
		testOnRandomPoints();
	
	// subscribe for point cloud data
	ros::Subscriber sub = n.subscribe< sensor_msgs::PointCloud2 >( "input_cloud", 1, &pointCloudCallback );	
	pub = n.advertise< visualization_msgs::MarkerArray >( "visualization_marker_array", 10 );
	
	ros::Rate loop_rate(100);
	while( n.ok() ) {
		ros::spinOnce();
		loop_rate.sleep();
	}
	
	
	return 0;
	
}




