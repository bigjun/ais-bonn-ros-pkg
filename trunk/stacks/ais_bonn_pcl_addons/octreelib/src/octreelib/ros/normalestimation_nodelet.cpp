/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 4/2011
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of University of Bonn, Computer Science Institute 
 *     VI nor the names of its contributors may be used to endorse or 
 *     promote products derived from this software without specific 
 *     prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *  implements pcl_ros::Feature 
 */


#include <pluginlib/class_list_macros.h>
#include "octreelib/ros/normalestimation_nodelet.h"

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

bool nodelet::NormalEstimationOctree::hasSubscribers() {
	for( unsigned int i = 0; i < pub_layer_output_.size(); i++ )
		if( pub_layer_output_[i].getNumSubscribers() > 0 )
			return true;
	return false;
}


inline bool nodelet::NormalEstimationOctree::childInit( ros::NodeHandle &nh ) {
	
	nh.param( "fixed_size", fixed_size_, false );
	nh.param( "min_resolution", min_resolution_, 0.005 );
	nh.param( "max_range", max_range_, 30.0 );
	nh.param( "min_points_for_fit", min_points_for_fit_, 25 );
	nh.param( "visualize_normals", visualize_normals_, false );
	
	allocator_ = NULL;
	
	// determine number of octree layers from min resolution and max range
	num_layers_ = ceilf( logf(2.f*max_range_ / min_resolution_) / logf(2.f) ) + 1; 
	
	pub_layer_output_ = std::vector< ros::Publisher >( num_layers_ );
	
	for( int i = 0; i < num_layers_; i++ ) {
		
		pub_layer_output_[i] = nh.advertise<PointCloudOut>( str( boost::format("output_layer%1%") % i ), max_queue_size_ );
		
	}
	
	if( visualize_normals_ )
		pub_normals_ = nh.advertise< visualization_msgs::MarkerArray >( "visualization_marker_array", max_queue_size_ );
		
	return true;
}


void nodelet::NormalEstimationOctree::emptyPublish( const PointCloudInConstPtr &cloud ) {
	PointCloudOut output;
	output.header = cloud->header;
	for( unsigned int i = 0; i < pub_layer_output_.size(); i++ )
		pub_layer_output_[i].publish( output.makeShared() );
}

void nodelet::NormalEstimationOctree::computePublish( const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices ) {
	
	// reallocate memory if configured to fixed size and if necessary
	// else reset allocator for reuse
	if( fixed_size_ ) {
		
		bool reallocate = false;
		if( !allocator_ ) {
			reallocate = true;
		}
		else {
			if( ((spatialaggregate::OcTreeNodeFixedCountAllocator< float, feature::NormalEstimationValue >*)allocator_)->numPoints < indices->size() ) {
				delete allocator_;
			}
		}
	
		if( reallocate )
			allocator_ = new spatialaggregate::OcTreeNodeFixedCountAllocator< float, feature::NormalEstimationValue >( indices->size() );
		else
			allocator_->reset();
		
	}
	
//	ros::Time startTime = ros::Time::now();
	
	// compute normals
	int octreeDepth = 0;
	algorithm::OcTreeSamplingMap< float, feature::NormalEstimationValue > octreeSamplingMap;
	
	boost::shared_ptr< spatialaggregate::OcTree<float, feature::NormalEstimationValue> > octree = feature::buildNormalEstimationOctree< float, feature::NormalEstimationValue, pcl::PointXYZ >( cloud, indices, octreeDepth, octreeSamplingMap, max_range_, min_resolution_, allocator_ );
	
	for( int i = 0; i <= octreeDepth; i++ ) {
		
		// no interest in layer => skip
		if( pub_layer_output_[i].getNumSubscribers() <= 0 )
			continue;
		
		feature::calculateNormalsAndCurvaturesOnOctreeLayer< float, feature::NormalEstimationValue >( octreeSamplingMap[i], min_points_for_fit_ );
	
		std::vector< spatialaggregate::OcTreeNode< float, feature::NormalEstimationValue >* >& nodeList = octreeSamplingMap[i];
		
		PointCloudOut output;
		output.header = cloud->header;
		output.points.reserve( nodeList.size() );
		output.width = nodeList.size();
		output.height = 1;
		output.is_dense = false;
		
		int numNormals = 0;
		for( unsigned int j = 0; j < nodeList.size(); j++ ) {
			
			spatialaggregate::OcTreeNode< float, feature::NormalEstimationValue >* node = nodeList[j];
			
			if( node->value.stable ) {
			
				const float invCount = 1.f / ((float)node->numPoints);
				
				Eigen::Vector3f summedPos = node->value.summedPos;
				summedPos *= invCount;

				const Eigen::Vector3f& normal = node->value.normal;
				
				pcl::PointNormal p;
				p.x = summedPos[0];
				p.y = summedPos[1];
				p.z = summedPos[2];
				p.normal[0] = normal[0];
				p.normal[1] = normal[1];
				p.normal[2] = normal[2];
				p.curvature = node->value.curvature;
				
				output.points.push_back( p );
				
				numNormals++;
				
			}
			
		}
		
		output.width = numNormals;
		
		pub_layer_output_[i].publish( output );
		
	}
	
//	ros::Time finishTime = ros::Time::now();
//	ROS_ERROR("octree normal estimation took %f for %i points", (finishTime-startTime).toSec(), indices ? indices->size() : cloud->points.size() );
	
	
	if( visualize_normals_ ) {
		
		visualization_msgs::MarkerArray markerArray;
		
		for( int i = 0; i <= octreeDepth; i++ ) {
			
			// no interest in layer => skip
			if( pub_layer_output_[i].getNumSubscribers() <= 0 )
				continue;
			
			visualization_msgs::Marker marker;
			marker.header = cloud->header;
			char str[255];
			sprintf( str, "normals %i", i );
			marker.ns = str;
			marker.id = 1;
			marker.type = visualization_msgs::Marker::LINE_LIST;
			marker.action = visualization_msgs::Marker::ADD;
			marker.points.clear();
			marker.scale.x = 0.002f * (float)(octreeDepth-i+1);
			marker.scale.y = 0.1f;
			marker.scale.z = 0.f;

			marker.color.r = 1.f;
			marker.color.g = 0.f;
			marker.color.b = 0.f;
			marker.color.a = 1.f;
			
			marker.lifetime = ros::Duration(0.0);
			
			for( int j = 0; j < octreeSamplingMap[i].size(); j++ ) {
				
				spatialaggregate::OcTreeNode< float, feature::NormalEstimationValue >* n = octreeSamplingMap[i][j];
				
				if( !n->value.stable )
					continue;
				
				float invCount = 1.f / ((float)n->numPoints);
				float nodeWidth = n->maxPosition[0] - n->minPosition[0];
				
				geometry_msgs::Point p1, p2;
				p1.x = n->value.summedPos[0] * invCount;
				p1.y = n->value.summedPos[1] * invCount;
				p1.z = n->value.summedPos[2] * invCount;
				p2.x = n->value.summedPos[0] * invCount + n->value.normal[0] * 2.f*nodeWidth;
				p2.y = n->value.summedPos[1] * invCount + n->value.normal[1] * 2.f*nodeWidth;
				p2.z = n->value.summedPos[2] * invCount + n->value.normal[2] * 2.f*nodeWidth;
				
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
		
		pub_normals_.publish( markerArray );
	}
}

typedef nodelet::NormalEstimationOctree NormalEstimationOctree;
PLUGINLIB_DECLARE_CLASS( octreelib, NormalEstimationOctree, NormalEstimationOctree, nodelet::Nodelet );

