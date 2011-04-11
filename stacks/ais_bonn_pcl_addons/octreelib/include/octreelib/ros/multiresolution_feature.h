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
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef __MULTIRESOLUTION_FEATURE_H__
#define __MULTIRESOLUTION_FEATURE_H__

#include <pcl/features/feature.h>
#include <pcl/PointIndices.h>

#include "pcl_ros/pcl_nodelet.h"
#include <message_filters/pass_through.h>

namespace pcl_ros {
	
	namespace sync_policies = message_filters::sync_policies;

	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	///////////////////////////////////////////////////////////////////////////////////////////
	/** \brief @b MultiResolutionFeature represents the base class for multi resolution feature. 
	* \author Joerg Stueckler
	*/
	class MultiResolutionFeature : public PCLNodelet
	{
	public:
		typedef pcl::PointCloud<pcl::PointXYZ> PointCloudIn;
		typedef PointCloudIn::Ptr PointCloudInPtr;
		typedef PointCloudIn::ConstPtr PointCloudInConstPtr;

		typedef boost::shared_ptr <std::vector<int> > IndicesPtr;
		typedef boost::shared_ptr <const std::vector<int> > IndicesConstPtr;

		/** \brief Empty constructor. */
		MultiResolutionFeature () {}

	protected:

		/** \brief The input PointCloud subscriber. */
		ros::Subscriber sub_input_;

		/** \brief Child initialization routine. Internal method. */
		virtual bool childInit( ros::NodeHandle &nh ) = 0;

		/** \brief Publish an empty point cloud of the feature output type. */
		virtual void emptyPublish( const PointCloudInConstPtr &cloud ) = 0;

		/** \brief Compute the feature and publish it. Internal method. */
		virtual void computePublish( const PointCloudInConstPtr &cloud, const IndicesConstPtr &indices ) = 0;

		/** \brief Null passthrough filter, used for pushing empty elements in the 
		* synchronizer */
		message_filters::PassThrough<PointIndices> nf_pi_;
		message_filters::PassThrough<PointCloudIn> nf_pc_;
		
		/** \brief Only start computing features when there are subscribers */
		virtual bool hasSubscribers() = 0;

		/** \brief Input point cloud callback.
		* Because we want to use the same synchronizer object, we push back
		* empty elements with the same timestamp.
		*/
		inline void input_callback( const PointCloudInConstPtr &input )
		{
			PointIndices indices;
			indices.header.stamp = input->header.stamp;
			PointCloudIn cloud;
			cloud.header.stamp = input->header.stamp;
			nf_pc_.add (cloud.makeShared ());
			nf_pi_.add (boost::make_shared<PointIndices> (indices));
		}

	private:
		/** \brief Synchronized input, and point indices.*/
		boost::shared_ptr< message_filters::Synchronizer< sync_policies::ApproximateTime< PointCloudIn, PointIndices > > > sync_input_indices_a_;
		boost::shared_ptr< message_filters::Synchronizer< sync_policies::ExactTime< PointCloudIn, PointIndices > > > sync_input_indices_e_;

		/** \brief Nodelet initialization routine. */
		virtual void onInit ();

		/** \brief Input point cloud callback. Used when \a use_indices and \a use_surface are set.
		* \param cloud the pointer to the input point cloud
		* \param indices the pointer to the input point cloud indices
		*/
		void input_indices_callback( const PointCloudInConstPtr &cloud, const PointIndicesConstPtr &indices );

	public:
		EIGEN_MAKE_ALIGNED_OPERATOR_NEW
	};

}

#endif  //#ifndef __MULTIRESOLUTION_FEATURE_H__
