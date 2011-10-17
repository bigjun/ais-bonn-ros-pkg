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
 */

#ifndef __SPATIALAGGREGATE_OCTREE_H__
#define __SPATIALAGGREGATE_OCTREE_H__

#include <list>
#include <vector>

#include <boost/shared_ptr.hpp>
#include <boost/make_shared.hpp>
#include <boost/enable_shared_from_this.hpp>

#include <Eigen/Core>
#include <Eigen/Eigen>

#include <ostream>

#include <stdint.h>

#define MAX_REPRESENTABLE_DEPTH 16



namespace spatialaggregate {
	
	template< typename CoordType, typename ValueType > class OcTreeNodeAllocator;
	template< typename CoordType, typename ValueType > class OcTree;
	
	enum OcTreeNodeType {
		OCTREE_LEAF_NODE,
		OCTREE_BRANCHING_NODE,
		NUM_OCTREE_NODE_TYPES,
	};
	
	
	template< class T >
	class DynamicAllocator {
	public:

		typedef typename std::list< std::vector< T, Eigen::aligned_allocator< T > > > Pool;
		typedef typename std::list< std::vector< T, Eigen::aligned_allocator< T > > >::iterator PoolIterator;

		DynamicAllocator( int block_size ) {

			block_size_ = block_size;
			pool_.push_back( std::vector< T, Eigen::aligned_allocator< T > >( block_size_ ) );
			pool_iterator_ = pool_.begin();
			curr_idx_ = 0;

		}

		~DynamicAllocator() {
		}

		T* allocate() {

			T* retval = &((*pool_iterator_)[curr_idx_]);

			curr_idx_++;
			if( curr_idx_ >= block_size_ ) {
				PoolIterator nit = pool_iterator_;
				nit++;
				if( nit == pool_.end() )
					pool_.push_back( std::vector< T, Eigen::aligned_allocator< T > >( block_size_ ) );
				pool_iterator_++;
				curr_idx_ = 0;
			}

			return retval;

		}

		void reset() {
			pool_iterator_ = pool_.begin();
			curr_idx_ = 0;
		}

		Pool pool_;
		int block_size_;
		int curr_idx_;
		PoolIterator pool_iterator_;


	};


	//! point in the octree with templated position and value type
	template< typename CoordType, typename ValueType >
	class OcTreeKey {
	public:

		OcTreeKey() {}

		OcTreeKey( const CoordType& x, const CoordType& y, const CoordType& z, OcTree< CoordType, ValueType >* tree ) {
			setKey( x, y, z, tree );
		}
		OcTreeKey( const Eigen::Matrix< CoordType, 4, 1 >& position, OcTree< CoordType, ValueType >* tree ) {
			setKey( position(0), position(1), position(2), tree );
		}

		~OcTreeKey() {}

		inline void setKey( const CoordType& x, const CoordType& y, const CoordType& z, OcTree< CoordType, ValueType >* tree ) {
			return setKey( Eigen::Matrix< CoordType, 4, 1 >( x, y, z, 1.0 ), tree );
		}

		inline void setKey( const Eigen::Matrix< CoordType, 4, 1 >& position, OcTree< CoordType, ValueType >* tree );


		inline bool operator==(const OcTreeKey &rhs) {
			return x_ == rhs.x_ && y_ == rhs.y_ && z_ == rhs.z_;
		}

		inline uint64_t dilate1By2( uint64_t x );
		inline uint64_t reduce1By2( uint64_t x );
		inline uint64_t encodeMorton48( uint64_t x, uint64_t y, uint64_t z );
		inline void decodeMorton48( uint64_t key, uint64_t& x, uint64_t& y, uint64_t& z );

		inline Eigen::Matrix< CoordType, 4, 1 > getPosition( OcTree< CoordType, ValueType >* tree ) const;

		uint32_t x_, y_, z_;

	public:
   	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	};
	
	
	/** \brief a node in the octree, either leaf or branching node
	 *
	 */
	template< typename CoordType, typename ValueType >
	class OcTreeNode {
	public:

		OcTreeNode() { 
			type_ = NUM_OCTREE_NODE_TYPES;
			initialize();
		}
		
		OcTreeNode( OcTreeNodeType type ) { 
			type_ = type;
			initialize();
		}

		OcTreeNode( OcTreeNodeType type, boost::shared_ptr< OcTree< CoordType, ValueType > > tree ) {
			type_ = type;
			tree_ = tree;
			initialize();
		}
		
		void initialize() {
			tree_ = NULL;
			parent_ = NULL;
			memset( children_, 0, sizeof( children_ ) );
			memset( neighbors_, 0, sizeof( neighbors_ ) );
			depth_ = 0;
			value_ = ValueType();
		}
		
		void initialize( OcTreeNode* node ) {
			tree_ = node->tree_;
			parent_ = node->parent_;
			depth_ = node->depth_;
			value_ = node->value_;
			type_ = node->type_;
			pos_key_ = node->pos_key_;
			max_key_ = node->max_key_;
			min_key_ = node->min_key_;

			memcpy( children_, node->children_, sizeof( children_ ) );
			memcpy( neighbors_, node->neighbors_, sizeof( neighbors_ ) );
		}

		inline void initialize( OcTreeNodeType type, const OcTreeKey< CoordType, ValueType >& key, const ValueType& value, int depth, OcTreeNode< CoordType, ValueType >* parent, OcTree< CoordType, ValueType >* tree );


		~OcTreeNode() {
			for( unsigned int i = 0; i < 8; i++ ) {
				if( children_[i] ) {
					tree_->allocator_->deallocateNode( children_[i] );
					children_[i] = NULL;
				}
			}
		}
		

		inline CoordType resolution();

		inline Eigen::Matrix< CoordType, 4, 1 > getPosition() const {
			return pos_key_.getPosition( tree_ );
		}

		OcTreeNodeType type_;

		OcTree< CoordType, ValueType >* tree_;
		OcTreeNode< CoordType, ValueType >* parent_;
		OcTreeNode< CoordType, ValueType >* children_[8];
		OcTreeNode< CoordType, ValueType >* neighbors_[27];

		OcTreeKey< CoordType, ValueType > pos_key_, min_key_, max_key_;
		int depth_;
		ValueType value_;
		

		//! position in my region?
		inline bool inRegion( const OcTreeKey< CoordType, ValueType >& position );
		
		//! my center in given region?
		inline bool inRegion( const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition );
		
		inline bool overlap( const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition );
		
		inline bool containedInRegion( const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition );
		
		inline bool regionContained( const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition );
		
		inline unsigned int getOctant( const OcTreeKey< CoordType, ValueType >& position );
		
		inline void getCenterKey( OcTreeKey< CoordType, ValueType >& center_key );

		inline OcTreeNode< CoordType, ValueType >* addPoint( const OcTreeKey< CoordType, ValueType >& position, const ValueType& value, int maxDepth );
		
		inline void getAllLeaves( std::list< OcTreeNode< CoordType, ValueType >* >& nodes );

		inline void getAllLeavesInVolume( std::list< OcTreeNode< CoordType, ValueType >* >& nodes, const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition, int maxDepth );

		inline void getAllNodesInVolumeOnDepth( std::list< OcTreeNode< CoordType, ValueType >* >& nodes, const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition, int depth, bool lowerDepthLeaves );

		inline ValueType getValueInVolume( const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition, int maxDepth );
		
		inline void applyOperatorInVolume( ValueType& value, void* data, void (*f)( ValueType& v, OcTreeNode< CoordType, ValueType >* current, void* data ), const OcTreeKey< CoordType, ValueType >& minPosition, const OcTreeKey< CoordType, ValueType >& maxPosition, int maxDepth );
		
		// sweeps up the tree and applies the given function on parent node and this node
		inline void sweepUp( void* data, void (*f)( OcTreeNode< CoordType, ValueType >* current, OcTreeNode< CoordType, ValueType >* next, void* data ) );
		
		inline void sweepDown( void* data, void (*f)( OcTreeNode< CoordType, ValueType >* current, OcTreeNode< CoordType, ValueType >* next, void* data ) );
		
		inline OcTreeNode< CoordType, ValueType >* findRepresentative( const OcTreeKey< CoordType, ValueType >& position, int maxDepth );

		inline unsigned int countNodes() {
			unsigned int numNodes = 1;
			for( unsigned int i = 0; i < 8; i++ )
				if( children_[i] )
					numNodes += children_[i]->countNodes();
			return numNodes;
		}

		inline void finishBranch();

		inline void establishNeighbors();


	public:
   	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	};
	
	
	/** \brief simple allocator uses new operator
	 *
	 */
	template< typename CoordType, typename ValueType >
	class OcTreeNodeAllocator : public boost::enable_shared_from_this< OcTreeNodeAllocator< CoordType, ValueType > > {
	public:
		
		OcTreeNodeAllocator() {}
		virtual ~OcTreeNodeAllocator() {}
		
		inline virtual OcTreeNode< CoordType, ValueType >* allocateNode() { return new OcTreeNode< CoordType, ValueType >(); }
		inline virtual void deallocateNode( OcTreeNode< CoordType, ValueType >* node ) { delete node; }
		
		inline virtual void reset() {}

	public:
   	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	};
	
	
	/** \brief fixed count allocator, pre-allocates memory for a fixed number of points
	 *
	 */
	template< typename CoordType, typename ValueType >
	class OcTreeNodeFixedCountAllocator : public OcTreeNodeAllocator< CoordType, ValueType > {
	public:
		
		OcTreeNodeFixedCountAllocator( unsigned int numPoints ) {
			const int numNodes = numPoints + (int)ceil(8.0*numPoints / 7.0);
			nodes_ = new OcTreeNode< CoordType, ValueType >[ numNodes ];
			current_node_ = &nodes_[0];
			last_node_ = &nodes_[numNodes-1];
		}
		
		virtual ~OcTreeNodeFixedCountAllocator() {
			delete[] nodes_;
		}
		
		inline virtual OcTreeNode< CoordType, ValueType >* allocateNode() {
			current_node_++;
			assert( current_node_ != last_node_ );
			return current_node_;
		}
		inline virtual void deallocateNode( OcTreeNode< CoordType, ValueType >* node ) {}
		
		inline virtual void reset() {
			OcTreeNode< CoordType, ValueType >* lastNode = current_node_;
			for( current_node_ = &nodes_[0]; current_node_ != lastNode; current_node_++ )
				current_node_->initialize();
			current_node_ = &nodes_[0];
		}
		
		OcTreeNode< CoordType, ValueType >* nodes_;
		OcTreeNode< CoordType, ValueType >* current_node_;
		OcTreeNode< CoordType, ValueType >* last_node_;

	public:
   	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	};

	
	/** \brief dynamic allocator, allocates memory in chunks
	 *
	 */
	template< typename CoordType, typename ValueType >
	class OcTreeNodeDynamicAllocator : public OcTreeNodeAllocator< CoordType, ValueType > {
	public:

		typedef typename DynamicAllocator< OcTreeNode< CoordType, ValueType > >::PoolIterator AllocPoolIterator;

		OcTreeNodeDynamicAllocator( unsigned int block_size )
		: alloc_( block_size ) {
		}

		virtual ~OcTreeNodeDynamicAllocator() {
		}

		inline virtual OcTreeNode< CoordType, ValueType >* allocateNode() {
			return alloc_.allocate();
		}

		inline virtual void deallocateNode( OcTreeNode< CoordType, ValueType >* node ) {}

		inline virtual void reset() {
			for( AllocPoolIterator it = alloc_.pool_.begin(); it != alloc_.pool_.end(); it++ ) {
				AllocPoolIterator nit = it;
				nit++;
				if( nit != alloc_.pool_.end()  ) {
					for( unsigned int j = 0; j < alloc_.block_size_; j++ )
						(*it)[j].initialize();
				}
				else {
					for( unsigned int j = 0; j < alloc_.curr_idx_; j++ )
						(*it)[j].initialize();
				}
			}
			alloc_.reset();
		}

		DynamicAllocator< OcTreeNode< CoordType, ValueType > > alloc_;

	public:
   	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	};


	/** \brief the octree class with some convenient constructors
	 *
	 */
	template< typename CoordType, typename ValueType >
	class OcTree {
	public:
		
		//! creates a tree in which the node sizes are multiples of minimumVolumeSize with maximal depth levels around center
		OcTree( const Eigen::Matrix< CoordType, 4, 1 >& center, CoordType minimumVolumeSize, CoordType maxDistance, boost::shared_ptr< OcTreeNodeAllocator< CoordType, ValueType > > allocator = boost::make_shared< OcTreeNodeAllocator< CoordType, ValueType > >() );
		
		~OcTree();
		
		inline void initialize( const Eigen::Matrix< CoordType, 4, 1 >& dimensions, const Eigen::Matrix< CoordType, 4, 1 >& center, CoordType minimumVolumeSize );
		
		inline OcTreeNode< CoordType, ValueType >* addPoint( const CoordType& x, const CoordType& y, const CoordType& z, const ValueType& value, int maxDepth ) {
			return root_->addPoint( getKey( x, y, z ), value, maxDepth );
		}

		inline OcTreeNode< CoordType, ValueType >* addPoint( const Eigen::Matrix< CoordType, 4, 1 >& position, const ValueType& value, int maxDepth ) {
			return root_->addPoint( getKey( position(0), position(1), position(2) ), value, maxDepth );
		}

		inline OcTreeKey< CoordType, ValueType > getKey( const CoordType& x, const CoordType& y, const CoordType& z ) {
			return OcTreeKey< CoordType, ValueType >( x, y, z, this );
		}

		inline OcTreeKey< CoordType, ValueType > getKey( const Eigen::Matrix< CoordType, 4, 1 >& position ) {
			return OcTreeKey< CoordType, ValueType >( position(0), position(1), position(2), this );
		}
		
		inline double depthForVolumeSize( CoordType volumeSize ) {
			return std::min( (double)max_depth_, std::max( 0.0, (double)max_depth_ - (log( volumeSize ) - log_minimum_volume_size_) * log2_inv_ ) );
		}

		inline CoordType volumeSizeForDepth( int depth ) {
			return resolutions_[depth];
		}

		inline void getAllNodesInVolumeOnDepth( std::list< OcTreeNode< CoordType, ValueType >* >& nodes, const Eigen::Matrix< CoordType, 4, 1 >& minPosition, const Eigen::Matrix< CoordType, 4, 1 >& maxPosition, int depth, bool lowerDepthLeaves ) {
			root_->getAllNodesInVolumeOnDepth( nodes, getKey( minPosition ), getKey( maxPosition ), depth, lowerDepthLeaves );
		}

		inline OcTreeNode< CoordType, ValueType >* findRepresentative( const Eigen::Matrix< CoordType, 4, 1 >& position, int maxDepth ) {
			return root_->findRepresentative( getKey( position ), maxDepth );
		}

		OcTreeNode< CoordType, ValueType >* root_;
		
		CoordType minimum_volume_size_;
		double log_minimum_volume_size_;
		double log2_inv_;
		int max_depth_;
		float resolutions_[MAX_REPRESENTABLE_DEPTH+1];
		uint32_t depth_masks_[MAX_REPRESENTABLE_DEPTH+1];
		uint32_t minmasks_[MAX_REPRESENTABLE_DEPTH+1];
		uint32_t maxmasks_[MAX_REPRESENTABLE_DEPTH+1];
		uint32_t neighbor_octant_[8][27];
		uint32_t parent_neighbor_[8][27];
		
		// required to generate keys
		Eigen::Matrix< CoordType, 4, 1 > min_position_, position_normalizer_;

		boost::shared_ptr< OcTreeNodeAllocator< CoordType, ValueType > > allocator_;

	public:
   	    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
	};

};


#include <octreelib/spatialaggregate/octree.hpp>


#endif //__SPATIALAGGREGATE_OCTREE_H__


