/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, University of Bonn, Computer Science Institute VI
 *  Author: Joerg Stueckler, 4/2011
 *  All rights reserved.
 *
 *  Copyright (c) 2009, Willow Garage, Inc.
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
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNERff OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id: feature.cpp 35422 2011-01-24 20:04:44Z rusu $
 *
 */

#include "octreelib/ros/multiresolution_feature.h"
#include <message_filters/null_types.h>

////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::MultiResolutionFeature::onInit () {
	// Call the super onInit ()
	PCLNodelet::onInit ();

	// Call the child init
	childInit (*pnh_);

	// Allow each individual class that inherits from us to declare their own Publisher
	// This is useful for Publisher<pcl::PointCloud<T> >, as NormalEstimation can publish PointCloud<Normal>, etc
	//pub_output_ = pnh_->template advertise<PointCloud2> ("output", max_queue_size_);


	// If we're supposed to look for PointIndices (indices)
	if( use_indices_ ) {
		
		// Create the objects here
		if( approximate_sync_ )
			sync_input_indices_a_ = boost::make_shared<message_filters::Synchronizer<sync_policies::ApproximateTime<PointCloudIn, PointIndices> > >(max_queue_size_);
		else
			sync_input_indices_e_ = boost::make_shared<message_filters::Synchronizer<sync_policies::ExactTime<PointCloudIn, PointIndices> > >(max_queue_size_);

		// Subscribe to the input using a filter
		sub_input_filter_.subscribe( *pnh_, "input", max_queue_size_ );
		sub_indices_filter_.subscribe( *pnh_, "indices", max_queue_size_ );
		if (approximate_sync_)
			sync_input_indices_a_->connectInput( sub_input_filter_, sub_indices_filter_ );
		else
			sync_input_indices_e_->connectInput( sub_input_filter_, sub_indices_filter_ );
	}
	else
		// Subscribe in an old fashion to input only (no filters)
		sub_input_ = pnh_->subscribe<PointCloudIn> ("input", max_queue_size_,  bind (&MultiResolutionFeature::input_indices_callback, this, _1, PointIndicesConstPtr ()));

}


////////////////////////////////////////////////////////////////////////////////////////////
void pcl_ros::MultiResolutionFeature::input_indices_callback( const PointCloudInConstPtr& cloud, const PointIndicesConstPtr& indices ) {

	// No subscribers, no work
	if( !hasSubscribers() )
		return;

	// If cloud is given, check if it's valid
	if( !isValid( cloud ) ) {
		
		NODELET_ERROR ("[%s::input_surface_indices_callback] Invalid input!", getName ().c_str ());
		emptyPublish (cloud);
		return;
		
	}

	// If indices are given, check if they are valid
	if( indices && !isValid (indices) ) {
		
		NODELET_ERROR ("[%s::input_surface_indices_callback] Invalid input indices!", getName ().c_str ());
		emptyPublish (cloud);
		return;
		
	}

	/// DEBUG
	if( indices )
		NODELET_DEBUG ("[input_indices_callback]\n"
					"                                 - PointCloud with %d data points (%s), stamp %f, and frame %s on topic %s received.\n"
					"                                 - PointIndices with %zu values, stamp %f, and frame %s on topic %s received.",
					cloud->width * cloud->height, pcl::getFieldsList (*cloud).c_str (), cloud->header.stamp.toSec (), cloud->header.frame_id.c_str (), pnh_->resolveName ("input").c_str (),
					indices->indices.size (), indices->header.stamp.toSec (), indices->header.frame_id.c_str (), pnh_->resolveName ("indices").c_str ());
	else
		NODELET_DEBUG ("[input_indices_callback] PointCloud with %d data points, stamp %f, and frame %s on topic %s received.", cloud->width * cloud->height, cloud->header.stamp.toSec (), cloud->header.frame_id.c_str (), pnh_->resolveName ("input").c_str ());
	///


	// If indices given...
	IndicesConstPtr vindices;
	if( indices && !indices->header.frame_id.empty() )
		vindices = boost::make_shared <const std::vector<int> >( indices->indices );

	computePublish( cloud, vindices );
}

