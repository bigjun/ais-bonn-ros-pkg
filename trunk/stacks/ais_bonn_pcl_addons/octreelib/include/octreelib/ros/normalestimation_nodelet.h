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

#ifndef __NORMALESTIMATION_OCTREE_NODELET_H__
#define __NORMALESTIMATION_OCTREE_NODELET_H__

#include "octreelib/ros/multiresolution_feature.h"

#include <boost/format.hpp>

#include <octreelib/spatialaggregate/octree.h>
#include <octreelib/feature/normalestimation.h>


namespace nodelet {
	
	/** \brief @b NormalEstimation estimates local surface properties at each 3D point, such as surface normals and
	* curvatures.
	*
	* \author Joerg Stueckler
	*/
	class NormalEstimationOctree : public pcl_ros::MultiResolutionFeature {
		
	protected:
		
		typedef pcl::PointCloud<pcl::PointNormal> PointCloudOut;
		
		bool hasSubscribers();
		
		/** \brief Child initialization routine. Internal method. */
		inline bool childInit( ros::NodeHandle &nh );
		
		/** \brief Publish an empty point cloud of the feature output type.
		* \param cloud the input point cloud to copy the header from.
		*/ 
		void emptyPublish( const PointCloudInConstPtr &cloud );

		/** \brief Compute the feature and publish it. */
		void computePublish( const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices );
		
		
		bool fixed_size_;
		double min_resolution_;
		double max_range_;
		int min_points_for_fit_;
		bool visualize_normals_;
		
		int num_layers_;
		
		std::vector< ros::Publisher > pub_layer_output_;
		ros::Publisher pub_normals_;
		
		spatialaggregate::OcTreeNodeAllocator< float, feature::NormalEstimationValue >* allocator_;
		
		
		
	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};
}

#endif  //__NORMALESTIMATION_OCTREE_NODELET_H__

