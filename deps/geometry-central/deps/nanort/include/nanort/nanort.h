//
// NanoRT, single header only modern ray tracing kernel.
//

/*
The MIT License (MIT)

Copyright (c) 2015 - 2016 Light Transport Entertainment, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef NANORT_H_
#define NANORT_H_

#include <algorithm>
#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <vector>

namespace nanort {

// Parallelized BVH build is not yet fully tested,
// thus turn off if you face a problem when building BVH.
#define NANORT_ENABLE_PARALLEL_BUILD (1)

	// ----------------------------------------------------------------------------
	// Small vector class useful for multi-threaded environment.
	//
	// stack_container.h
	//
	// Copyright (c) 2006-2008 The Chromium Authors. All rights reserved.
	// Use of this source code is governed by a BSD-style license that can be
	// found in the LICENSE file.

	// This allocator can be used with STL containers to provide a stack buffer
	// from which to allocate memory and overflows onto the heap. This stack buffer
	// would be allocated on the stack and allows us to avoid heap operations in
	// some situations.
	//
	// STL likes to make copies of allocators, so the allocator itself can't hold
	// the data. Instead, we make the creator responsible for creating a
	// StackAllocator::Source which contains the data. Copying the allocator
	// merely copies the pointer to this shared source, so all allocators created
	// based on our allocator will share the same stack buffer.
	//
	// This stack buffer implementation is very simple. The first allocation that
	// fits in the stack buffer will use the stack buffer. Any subsequent
	// allocations will not use the stack buffer, even if there is unused room.
	// This makes it appropriate for array-like containers, but the caller should
	// be sure to reserve() in the container up to the stack buffer size. Otherwise
	// the container will allocate a small array which will "use up" the stack
	// buffer.
	template<typename T, size_t stack_capacity>
	class StackAllocator : public std::allocator<T> {
	public:
		typedef typename std::allocator<T>::pointer pointer;
		typedef typename std::allocator<T>::size_type size_type;

		// Backing store for the allocator. The container owner is responsible for
		// maintaining this for as long as any containers using this allocator are
		// live.
		struct Source {
			Source() : used_stack_buffer_(false) {}

			// Casts the buffer in its right type.
			T *stack_buffer() { return reinterpret_cast<T *>(stack_buffer_); }
			const T *stack_buffer() const { return reinterpret_cast<const T *>(stack_buffer_); }

			//
			// IMPORTANT: Take care to ensure that stack_buffer_ is aligned
			// since it is used to mimic an array of T.
			// Be careful while declaring any unaligned types (like bool)
			// before stack_buffer_.
			//

			// The buffer itself. It is not of type T because we don't want the
			// constructors and destructors to be automatically called. Define a POD
			// buffer of the right size instead.
			char stack_buffer_[sizeof(T[stack_capacity])];

			// Set when the stack buffer is used for an allocation. We do not track
			// how much of the buffer is used, only that somebody is using it.
			bool used_stack_buffer_;
		};

		// Used by containers when they want to refer to an allocator of type U.
		template<typename U>
		struct rebind {
			typedef StackAllocator<U, stack_capacity> other;
		};

		// For the straight up copy c-tor, we can share storage.
		StackAllocator(const StackAllocator<T, stack_capacity> &rhs) : source_(rhs.source_) {}

		// ISO C++ requires the following constructor to be defined,
		// and std::vector in VC++2008SP1 Release fails with an error
		// in the class _Container_base_aux_alloc_real (from <xutility>)
		// if the constructor does not exist.
		// For this constructor, we cannot share storage; there's
		// no guarantee that the Source buffer of Ts is large enough
		// for Us.
		// TODO(Google): If we were fancy pants, perhaps we could share storage
		// iff sizeof(T) == sizeof(U).
		template<typename U, size_t other_capacity>
		StackAllocator(const StackAllocator<U, other_capacity> &other) : source_(NULL) {
			(void) other;
		}

		explicit StackAllocator(Source *source) : source_(source) {}

		// Actually do the allocation. Use the stack buffer if nobody has used it yet
		// and the size requested fits. Otherwise, fall through to the standard
		// allocator.
		pointer allocate(size_type n, void *hint = 0) {
			if (source_ != NULL && !source_->used_stack_buffer_ && n <= stack_capacity) {
				source_->used_stack_buffer_ = true;
				return source_->stack_buffer();
			} else {
				return std::allocator<T>::allocate(n, hint);
			}
		}

		// Free: when trying to free the stack buffer, just mark it as free. For
		// non-stack-buffer pointers, just fall though to the standard allocator.
		void deallocate(pointer p, size_type n) {
			if (source_ != NULL && p == source_->stack_buffer())
				source_->used_stack_buffer_ = false;
			else
				std::allocator<T>::deallocate(p, n);
		}

	private:
		Source *source_;
	};

	// A wrapper around STL containers that maintains a stack-sized buffer that the
	// initial capacity of the vector is based on. Growing the container beyond the
	// stack capacity will transparently overflow onto the heap. The container must
	// support reserve().
	//
	// WATCH OUT: the ContainerType MUST use the proper StackAllocator for this
	// type. This object is really intended to be used only internally. You'll want
	// to use the wrappers below for different types.
	template<typename TContainerType, int stack_capacity>
	class StackContainer {
	public:
		typedef TContainerType ContainerType;
		typedef typename ContainerType::value_type ContainedType;
		typedef StackAllocator<ContainedType, stack_capacity> Allocator;

		// Allocator must be constructed before the container!
		StackContainer() : allocator_(&stack_data_), container_(allocator_) {
			// Make the container use the stack allocation by reserving our buffer size
			// before doing anything else.
			container_.reserve(stack_capacity);
		}

		// Getters for the actual container.
		//
		// Danger: any copies of this made using the copy constructor must have
		// shorter lifetimes than the source. The copy will share the same allocator
		// and therefore the same stack buffer as the original. Use std::copy to
		// copy into a "real" container for longer-lived objects.
		ContainerType &container() { return container_; }
		const ContainerType &container() const { return container_; }

		// Support operator-> to get to the container. This allows nicer syntax like:
		//   StackContainer<...> foo;
		//   std::sort(foo->begin(), foo->end());
		ContainerType *operator->() { return &container_; }
		const ContainerType *operator->() const { return &container_; }

#ifdef UNIT_TEST
		// Retrieves the stack source so that that unit tests can verify that the
		// buffer is being used properly.
		const typename Allocator::Source &stack_data() const { return stack_data_; }
#endif

	protected:
		typename Allocator::Source stack_data_;
		unsigned char pad_[7];
		Allocator allocator_;
		ContainerType container_;

		// DISALLOW_EVIL_CONSTRUCTORS(StackContainer);
		StackContainer(const StackContainer &);
		void operator=(const StackContainer &);
	};

	// StackVector
	//
	// Example:
	//   StackVector<int, 16> foo;
	//   foo->push_back(22);  // we have overloaded operator->
	//   foo[0] = 10;         // as well as operator[]
	template<typename T, size_t stack_capacity>
	class StackVector : public StackContainer<std::vector<T, StackAllocator<T, stack_capacity> >, stack_capacity> {
	public:
		StackVector() : StackContainer<std::vector<T, StackAllocator<T, stack_capacity> >, stack_capacity>() {}

		// We need to put this in STL containers sometimes, which requires a copy
		// constructor. We can't call the regular copy constructor because that will
		// take the stack buffer from the original. Here, we create an empty object
		// and make a stack buffer of its own.
		StackVector(const StackVector<T, stack_capacity> &other) :
			StackContainer<std::vector<T, StackAllocator<T, stack_capacity> >, stack_capacity>() {
			this->container().assign(other->begin(), other->end());
		}

		StackVector<T, stack_capacity> &operator=(const StackVector<T, stack_capacity> &other) {
			this->container().assign(other->begin(), other->end());
			return *this;
		}

		// Vectors are commonly indexed, which isn't very convenient even with
		// operator-> (using "->at()" does exception stuff we don't want).
		T &operator[](size_t i) { return this->container().operator[](i); }
		const T &operator[](size_t i) const { return this->container().operator[](i); }
	};

	// ----------------------------------------------------------------------------

	template<typename T = float>
	class real3 {
	public:
		real3() {}
		real3(T x) {
			v[0] = x;
			v[1] = x;
			v[2] = x;
		}
		real3(T xx, T yy, T zz) {
			v[0] = xx;
			v[1] = yy;
			v[2] = zz;
		}
		explicit real3(const T *p) {
			v[0] = p[0];
			v[1] = p[1];
			v[2] = p[2];
		}

		inline T x() const { return v[0]; }
		inline T y() const { return v[1]; }
		inline T z() const { return v[2]; }

		real3 operator*(T f) const { return real3(x() * f, y() * f, z() * f); }
		real3 operator-(const real3 &f2) const { return real3(x() - f2.x(), y() - f2.y(), z() - f2.z()); }
		real3 operator*(const real3 &f2) const { return real3(x() * f2.x(), y() * f2.y(), z() * f2.z()); }
		real3 operator+(const real3 &f2) const { return real3(x() + f2.x(), y() + f2.y(), z() + f2.z()); }
		real3 &operator+=(const real3 &f2) {
			v[0] += f2.x();
			v[1] += f2.y();
			v[2] += f2.z();
			return (*this);
		}
		real3 operator/(const real3 &f2) const { return real3(x() / f2.x(), y() / f2.y(), z() / f2.z()); }
		real3 operator-() const { return real3(-x(), -y(), -z()); }
		T operator[](int i) const { return v[i]; }
		T &operator[](int i) { return v[i]; }

		T v[3];
		// T pad;  // for alignment(when T = float)
	};

	template<typename T>
	inline real3<T> operator*(T f, const real3<T> &v) {
		return real3<T>(v.x() * f, v.y() * f, v.z() * f);
	}

	template<typename T>
	inline real3<T> vneg(const real3<T> &rhs) {
		return real3<T>(-rhs.x(), -rhs.y(), -rhs.z());
	}

	template<typename T>
	inline T vlength(const real3<T> &rhs) {
		return std::sqrt(rhs.x() * rhs.x() + rhs.y() * rhs.y() + rhs.z() * rhs.z());
	}

	template<typename T>
	inline real3<T> vnormalize(const real3<T> &rhs) {
		real3<T> v = rhs;
		T len = vlength(rhs);
		if (fabs(len) > 1.0e-6f) {
			float inv_len = 1.0f / len;
			v.v[0] *= inv_len;
			v.v[1] *= inv_len;
			v.v[2] *= inv_len;
		}
		return v;
	}

	template<typename T>
	inline real3<T> vcross(real3<T> a, real3<T> b) {
		real3<T> c;
		c[0] = a[1] * b[2] - a[2] * b[1];
		c[1] = a[2] * b[0] - a[0] * b[2];
		c[2] = a[0] * b[1] - a[1] * b[0];
		return c;
	}

	template<typename T>
	inline T vdot(real3<T> a, real3<T> b) {
		return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
	}

	template<typename real>
	inline const real *get_vertex_addr(const real *p, const size_t idx, const size_t stride_bytes) {
		return reinterpret_cast<const real *>(reinterpret_cast<const unsigned char *>(p) + idx * stride_bytes);
	}

	template<typename real = float>
	class Ray {
	public:
		real org[3];	  // must set
		real dir[3];	  // must set
		real min_t;		  // minium ray hit distance. must set.
		real max_t;		  // maximum ray hit distance. must set.
		real inv_dir[3];  // filled internally
		int dir_sign[3];  // filled internally
	};

	template<typename real = float>
	class BVHNode {
	public:
		BVHNode() {}
		~BVHNode() {}

		real bmin[3];
		real bmax[3];

		int flag;  // 1 = leaf node, 0 = branch node
		int axis;

		// leaf
		//   data[0] = npoints
		//   data[1] = index
		//
		// branch
		//   data[0] = child[0]
		//   data[1] = child[1]
		unsigned int data[2];
	};

	template<class I>
	class IsectComparator {
	public:
		bool operator()(const I &a, const I &b) const { return a.t < b.t; }
	};

	/// BVH build option.
	template<typename T = float>
	struct BVHBuildOptions {
		T cost_t_aabb;
		unsigned int min_leaf_primitives;
		unsigned int max_tree_depth;
		unsigned int bin_size;
		unsigned int shallow_depth;
		unsigned int min_primitives_for_parallel_build;

		// Cache bounding box computation.
		// Requires more memory, but BVHbuild can be faster.
		bool cache_bbox;
		unsigned char pad[3];

		// Set default value: Taabb = 0.2
		BVHBuildOptions() :
			cost_t_aabb(0.2f),
			min_leaf_primitives(4),
			max_tree_depth(256),
			bin_size(64),
			shallow_depth(3),
			min_primitives_for_parallel_build(1024 * 128),
			cache_bbox(false) {}
	};

	/// BVH build statistics.
	class BVHBuildStatistics {
	public:
		unsigned int max_tree_depth;
		unsigned int num_leaf_nodes;
		unsigned int num_branch_nodes;
		float build_secs;

		// Set default value: Taabb = 0.2
		BVHBuildStatistics() : max_tree_depth(0), num_leaf_nodes(0), num_branch_nodes(0), build_secs(0.0f) {}
	};

	/// BVH trace option.
	class BVHTraceOptions {
	public:
		// Hit only for face IDs in indexRange.
		// This feature is good to mimic something like glDrawArrays()
		unsigned int prim_ids_range[2];
		bool cull_back_face;
		unsigned char pad[3];  ///< Padding(not used)

		BVHTraceOptions() {
			prim_ids_range[0] = 0;
			prim_ids_range[1] = 0x7FFFFFFF;	 // Up to 2G face IDs.
			cull_back_face = false;
		}
	};

	template<typename T>
	class BBox {
	public:
		real3<T> bmin;
		real3<T> bmax;

		BBox() {
			bmin[0] = bmin[1] = bmin[2] = std::numeric_limits<T>::max();
			bmax[0] = bmax[1] = bmax[2] = -std::numeric_limits<T>::max();
		}
	};

	template<typename T, class P, class Pred, class I>
	class BVHAccel {
	public:
		BVHAccel() : pad0_(0) { (void) pad0_; }
		~BVHAccel() {}

		/// Build BVH for input primitives.
		bool Build(const unsigned int num_primitives, const BVHBuildOptions<T> &options, const P &p, const Pred &pred);

		/// Get statistics of built BVH tree. Valid after Build()
		BVHBuildStatistics GetStatistics() const { return stats_; }

		/// Dump built BVH to the file.
		bool Dump(const char *filename);

		/// Load BVH binary
		bool Load(const char *filename);

		/// Traverse into BVH along ray and find closest hit point & primitive if
		/// found
		bool Traverse(const Ray<T> &ray, const BVHTraceOptions &options, const I &intersector) const;

		/// Multi-hit ray traversal
		/// Returns `max_intersections` frontmost intersections
		bool MultiHitTraverse(const Ray<T> &ray, const BVHTraceOptions &optins, int max_intersections, StackVector<I, 128> *intersector) const;

		const std::vector<BVHNode<T> > &GetNodes() const { return nodes_; }
		const std::vector<unsigned int> &GetIndices() const { return indices_; }

		/// Returns bounding box of built BVH.
		void BoundingBox(T bmin[3], T bmax[3]) const {
			if (nodes_.empty()) {
				bmin[0] = bmin[1] = bmin[2] = std::numeric_limits<T>::max();
				bmax[0] = bmax[1] = bmax[2] = -std::numeric_limits<T>::max();
			} else {
				bmin[0] = nodes_[0].bmin[0];
				bmin[1] = nodes_[0].bmin[1];
				bmin[2] = nodes_[0].bmin[2];
				bmax[0] = nodes_[0].bmax[0];
				bmax[1] = nodes_[0].bmax[1];
				bmax[2] = nodes_[0].bmax[2];
			}
		}

		bool IsValid() const { return nodes_.size() > 0; }

	private:
#if NANORT_ENABLE_PARALLEL_BUILD
		typedef struct {
			unsigned int left_idx;
			unsigned int right_idx;
			unsigned int offset;
		} ShallowNodeInfo;

		// Used only during BVH construction
		std::vector<ShallowNodeInfo> shallow_node_infos_;

		/// Builds shallow BVH tree recursively.
		unsigned int BuildShallowTree(std::vector<BVHNode<T> > *out_nodes, unsigned int left_idx, unsigned int right_idx, unsigned int depth,
									  unsigned int max_shallow_depth, const P &p, const Pred &pred);
#endif

		/// Builds BVH tree recursively.
		unsigned int BuildTree(BVHBuildStatistics *out_stat, std::vector<BVHNode<T> > *out_nodes, unsigned int left_idx, unsigned int right_idx,
							   unsigned int depth, const P &p, const Pred &pred);

		bool TestLeafNode(const BVHNode<T> &node, const Ray<T> &ray, const I &intersector) const;

		// bool MultiHitTestLeafNode(IsectVector *isects, int max_intersections,
		//                          const BVHNode &node, const Ray &ray,
		//                          const I &intersector) const;

		std::vector<BVHNode<T> > nodes_;
		std::vector<unsigned int> indices_;	 // max 4G triangles.
		std::vector<BBox<T> > bboxes_;
		BVHBuildOptions<T> options_;
		BVHBuildStatistics stats_;
		unsigned int pad0_;
	};

	// Predefined SAH predicator for triangle.
	template<typename T = float>
	class TriangleSAHPred {
	public:
		TriangleSAHPred(const T *vertices, const unsigned int *faces,
						size_t vertex_stride_bytes)	 // e.g. 12 for sizeof(float) * XYZ
			:
			axis_(0), pos_(0.0f), vertices_(vertices), faces_(faces), vertex_stride_bytes_(vertex_stride_bytes) {}

		void Set(int axis, T pos) const {
			axis_ = axis;
			pos_ = pos;
		}

		bool operator()(unsigned int i) const {
			int axis = axis_;
			T pos = pos_;

			unsigned int i0 = faces_[3 * i + 0];
			unsigned int i1 = faces_[3 * i + 1];
			unsigned int i2 = faces_[3 * i + 2];

			real3<T> p0(get_vertex_addr<T>(vertices_, i0, vertex_stride_bytes_));
			real3<T> p1(get_vertex_addr<T>(vertices_, i1, vertex_stride_bytes_));
			real3<T> p2(get_vertex_addr<T>(vertices_, i2, vertex_stride_bytes_));

			T center = p0[axis] + p1[axis] + p2[axis];

			return (center < pos * 3.0);
		}

	private:
		mutable int axis_;
		mutable T pos_;
		const T *vertices_;
		const unsigned int *faces_;
		const size_t vertex_stride_bytes_;
	};

	// Predefined Triangle mesh geometry.
	template<typename T = float>
	class TriangleMesh {
	public:
		TriangleMesh(const T *vertices, const unsigned int *faces,
					 const size_t vertex_stride_bytes)	// e.g. 12 for sizeof(float) * XYZ
			:
			vertices_(vertices), faces_(faces), vertex_stride_bytes_(vertex_stride_bytes) {}

		/// Compute bounding box for `prim_index`th triangle.
		/// This function is called for each primitive in BVH build.
		void BoundingBox(real3<T> *bmin, real3<T> *bmax, unsigned int prim_index) const {
			(*bmin)[0] = get_vertex_addr(vertices_, faces_[3 * prim_index + 0], vertex_stride_bytes_)[0];
			(*bmin)[1] = get_vertex_addr(vertices_, faces_[3 * prim_index + 0], vertex_stride_bytes_)[1];
			(*bmin)[2] = get_vertex_addr(vertices_, faces_[3 * prim_index + 0], vertex_stride_bytes_)[2];
			(*bmax)[0] = get_vertex_addr(vertices_, faces_[3 * prim_index + 0], vertex_stride_bytes_)[0];
			(*bmax)[1] = get_vertex_addr(vertices_, faces_[3 * prim_index + 0], vertex_stride_bytes_)[1];
			(*bmax)[2] = get_vertex_addr(vertices_, faces_[3 * prim_index + 0], vertex_stride_bytes_)[2];

			for (unsigned int i = 1; i < 3; i++) {
				for (unsigned int k = 0; k < 3; k++) {
					if ((*bmin)[static_cast<int>(k)] > get_vertex_addr<T>(vertices_, faces_[3 * prim_index + i], vertex_stride_bytes_)[k]) {
						(*bmin)[static_cast<int>(k)] = get_vertex_addr<T>(vertices_, faces_[3 * prim_index + i], vertex_stride_bytes_)[k];
					}
					if ((*bmax)[static_cast<int>(k)] < get_vertex_addr<T>(vertices_, faces_[3 * prim_index + i], vertex_stride_bytes_)[k]) {
						(*bmax)[static_cast<int>(k)] = get_vertex_addr<T>(vertices_, faces_[3 * prim_index + i], vertex_stride_bytes_)[k];
					}
				}
			}
		}

		const T *vertices_;
		const unsigned int *faces_;
		const size_t vertex_stride_bytes_;
	};

	template<typename T = float>
	class TriangleIntersection {
	public:
		T u;
		T v;

		// Required member variables.
		T t;
		unsigned int prim_id;
	};

	template<typename T = float, class I = TriangleIntersection<T> >
	class TriangleIntersector {
	public:
		TriangleIntersector(const T *vertices, const unsigned int *faces,
							const size_t vertex_stride_bytes)  // e.g.
															   // vertex_stride_bytes
															   // = 12 = sizeof(float)
															   // * 3
			:
			vertices_(vertices), faces_(faces), vertex_stride_bytes_(vertex_stride_bytes) {}

		// For Watertight Ray/Triangle Intersection.
		typedef struct {
			T Sx;
			T Sy;
			T Sz;
			int kx;
			int ky;
			int kz;
		} RayCoeff;

		/// Do ray interesection stuff for `prim_index` th primitive and return hit
		/// distance `t`,
		/// varycentric coordinate `u` and `v`.
		/// Returns true if there's intersection.
		bool Intersect(T *t_inout, unsigned int prim_index) const {
			if ((prim_index < trace_options_.prim_ids_range[0]) || (prim_index >= trace_options_.prim_ids_range[1])) {
				return false;
			}

			const unsigned int f0 = faces_[3 * prim_index + 0];
			const unsigned int f1 = faces_[3 * prim_index + 1];
			const unsigned int f2 = faces_[3 * prim_index + 2];

			const real3<T> p0(get_vertex_addr(vertices_, f0 + 0, vertex_stride_bytes_));
			const real3<T> p1(get_vertex_addr(vertices_, f1 + 0, vertex_stride_bytes_));
			const real3<T> p2(get_vertex_addr(vertices_, f2 + 0, vertex_stride_bytes_));

			const real3<T> A = p0 - ray_org_;
			const real3<T> B = p1 - ray_org_;
			const real3<T> C = p2 - ray_org_;

			const T Ax = A[ray_coeff_.kx] - ray_coeff_.Sx * A[ray_coeff_.kz];
			const T Ay = A[ray_coeff_.ky] - ray_coeff_.Sy * A[ray_coeff_.kz];
			const T Bx = B[ray_coeff_.kx] - ray_coeff_.Sx * B[ray_coeff_.kz];
			const T By = B[ray_coeff_.ky] - ray_coeff_.Sy * B[ray_coeff_.kz];
			const T Cx = C[ray_coeff_.kx] - ray_coeff_.Sx * C[ray_coeff_.kz];
			const T Cy = C[ray_coeff_.ky] - ray_coeff_.Sy * C[ray_coeff_.kz];

			T U = Cx * By - Cy * Bx;
			T V = Ax * Cy - Ay * Cx;
			T W = Bx * Ay - By * Ax;

			// Fall back to test against edges using double precision.
			if (U == 0.0 || V == 0.0 || W == 0.0) {
				double CxBy = static_cast<double>(Cx) * static_cast<double>(By);
				double CyBx = static_cast<double>(Cy) * static_cast<double>(Bx);
				U = static_cast<T>(CxBy - CyBx);

				double AxCy = static_cast<double>(Ax) * static_cast<double>(Cy);
				double AyCx = static_cast<double>(Ay) * static_cast<double>(Cx);
				V = static_cast<T>(AxCy - AyCx);

				double BxAy = static_cast<double>(Bx) * static_cast<double>(Ay);
				double ByAx = static_cast<double>(By) * static_cast<double>(Ax);
				W = static_cast<T>(BxAy - ByAx);
			}

			if (trace_options_.cull_back_face) {
				if (U < 0.0 || V < 0.0 || W < 0.0)
					return false;
			} else {
				if ((U < 0.0 || V < 0.0 || W < 0.0) && (U > 0.0 || V > 0.0 || W > 0.0)) {
					return false;
				}
			}

			T det = U + V + W;
			if (det == 0.0)
				return false;

			const T Az = ray_coeff_.Sz * A[ray_coeff_.kz];
			const T Bz = ray_coeff_.Sz * B[ray_coeff_.kz];
			const T Cz = ray_coeff_.Sz * C[ray_coeff_.kz];
			const T D = U * Az + V * Bz + W * Cz;

			const T rcpDet = static_cast<T>(1.0) / det;
			T tt = D * rcpDet;

			if (tt > (*t_inout)) {
				return false;
			}

			(*t_inout) = tt;
			// Use Thomas-Mueller style barycentric coord.
			// U + V + W = 1.0 and interp(p) = U * p0 + V * p1 + W * p2
			// We want interp(p) = (1 - u - v) * p0 + u * v1 + v * p2;
			// => u = V, v = W.
			intersection.u = V * rcpDet;
			intersection.v = W * rcpDet;

			return true;
		}

		/// Returns the nearest hit distance.
		T GetT() const { return intersection.t; }

		/// Update is called when initializing intesection and nearest hit is found.
		void Update(T t, unsigned int prim_idx) const {
			intersection.t = t;
			intersection.prim_id = prim_idx;
		}

		/// Prepare BVH traversal(e.g. compute inverse ray direction)
		/// This function is called only once in BVH traversal.
		void PrepareTraversal(const Ray<T> &ray, const BVHTraceOptions &trace_options) const {
			ray_org_[0] = ray.org[0];
			ray_org_[1] = ray.org[1];
			ray_org_[2] = ray.org[2];

			// Calculate dimension where the ray direction is maximal.
			ray_coeff_.kz = 0;
			T absDir = std::fabs(ray.dir[0]);
			if (absDir < std::fabs(ray.dir[1])) {
				ray_coeff_.kz = 1;
				absDir = std::fabs(ray.dir[1]);
			}
			if (absDir < std::fabs(ray.dir[2])) {
				ray_coeff_.kz = 2;
				absDir = std::fabs(ray.dir[2]);
			}

			ray_coeff_.kx = ray_coeff_.kz + 1;
			if (ray_coeff_.kx == 3)
				ray_coeff_.kx = 0;
			ray_coeff_.ky = ray_coeff_.kx + 1;
			if (ray_coeff_.ky == 3)
				ray_coeff_.ky = 0;

			// Swap kx and ky dimention to preserve widing direction of triangles.
			if (ray.dir[ray_coeff_.kz] < 0.0f)
				std::swap(ray_coeff_.kx, ray_coeff_.ky);

			// Claculate shear constants.
			ray_coeff_.Sx = ray.dir[ray_coeff_.kx] / ray.dir[ray_coeff_.kz];
			ray_coeff_.Sy = ray.dir[ray_coeff_.ky] / ray.dir[ray_coeff_.kz];
			ray_coeff_.Sz = 1.0f / ray.dir[ray_coeff_.kz];

			trace_options_ = trace_options;

			intersection.u = 0.0f;
			intersection.v = 0.0f;
		}

		/// Post BVH traversal stuff(e.g. compute intersection point information)
		/// This function is called only once in BVH traversal.
		/// `hit` = true if there is something hit.
		void PostTraversal(const Ray<T> &ray, bool hit) const {
			if (hit) {
				// Do something when there is a hit.
			}
			(void) ray;
		}

		const T *vertices_;
		const unsigned int *faces_;
		const size_t vertex_stride_bytes_;
		mutable real3<T> ray_org_;
		mutable RayCoeff ray_coeff_;
		mutable BVHTraceOptions trace_options_;

		mutable I intersection;
	};

	//
	// Robust BVH Ray Traversal : http://jcgt.org/published/0002/02/02/paper.pdf
	//

	// NaN-safe min and max function.
	template<class T>
	const T &safemin(const T &a, const T &b) {
		return (a < b) ? a : b;
	}
	template<class T>
	const T &safemax(const T &a, const T &b) {
		return (a > b) ? a : b;
	}

	//
	// SAH functions
	//
	struct BinBuffer {
		explicit BinBuffer(unsigned int size) {
			bin_size = size;
			bin.resize(2 * 3 * size);
			clear();
		}

		void clear() { memset(&bin[0], 0, sizeof(size_t) * 2 * 3 * bin_size); }

		std::vector<size_t> bin;  // (min, max) * xyz * binsize
		unsigned int bin_size;
		unsigned int pad0;
	};

	template<typename T>
	inline T CalculateSurfaceArea(const real3<T> &min, const real3<T> &max) {
		real3<T> box = max - min;
		return static_cast<T>(2.0) * (box[0] * box[1] + box[1] * box[2] + box[2] * box[0]);
	}

	template<typename T>
	inline void GetBoundingBoxOfTriangle(real3<T> *bmin, real3<T> *bmax, const T *vertices, const unsigned int *faces, unsigned int index) {
		unsigned int f0 = faces[3 * index + 0];
		unsigned int f1 = faces[3 * index + 1];
		unsigned int f2 = faces[3 * index + 2];

		real3<T> p[3];

		p[0] = real3<T>(&vertices[3 * f0]);
		p[1] = real3<T>(&vertices[3 * f1]);
		p[2] = real3<T>(&vertices[3 * f2]);

		(*bmin) = p[0];
		(*bmax) = p[0];

		for (int i = 1; i < 3; i++) {
			(*bmin)[0] = std::min((*bmin)[0], p[i][0]);
			(*bmin)[1] = std::min((*bmin)[1], p[i][1]);
			(*bmin)[2] = std::min((*bmin)[2], p[i][2]);

			(*bmax)[0] = std::max((*bmax)[0], p[i][0]);
			(*bmax)[1] = std::max((*bmax)[1], p[i][1]);
			(*bmax)[2] = std::max((*bmax)[2], p[i][2]);
		}
	}

	template<typename T, class P>
	inline void ContributeBinBuffer(BinBuffer *bins,  // [out]
									const real3<T> &scene_min, const real3<T> &scene_max, unsigned int *indices, unsigned int left_idx,
									unsigned int right_idx, const P &p) {
		T bin_size = static_cast<T>(bins->bin_size);

		// Calculate extent
		real3<T> scene_size, scene_inv_size;
		scene_size = scene_max - scene_min;
		for (int i = 0; i < 3; ++i) {
			assert(scene_size[i] >= 0.0);

			if (scene_size[i] > 0.0) {
				scene_inv_size[i] = bin_size / scene_size[i];
			} else {
				scene_inv_size[i] = 0.0;
			}
		}

		// Clear bin data
		std::fill(bins->bin.begin(), bins->bin.end(), 0);
		// memset(&bins->bin[0], 0, sizeof(2 * 3 * bins->bin_size));

		size_t idx_bmin[3];
		size_t idx_bmax[3];

		for (size_t i = left_idx; i < right_idx; i++) {
			//
			// Quantize the position into [0, BIN_SIZE)
			//
			// q[i] = (int)(p[i] - scene_bmin) / scene_size
			//
			real3<T> bmin;
			real3<T> bmax;

			p.BoundingBox(&bmin, &bmax, indices[i]);
			// GetBoundingBoxOfTriangle(&bmin, &bmax, vertices, faces, indices[i]);

			real3<T> quantized_bmin = (bmin - scene_min) * scene_inv_size;
			real3<T> quantized_bmax = (bmax - scene_min) * scene_inv_size;

			// idx is now in [0, BIN_SIZE)
			for (int j = 0; j < 3; ++j) {
				int q0 = static_cast<int>(quantized_bmin[j]);
				if (q0 < 0)
					q0 = 0;
				int q1 = static_cast<int>(quantized_bmax[j]);
				if (q1 < 0)
					q1 = 0;

				idx_bmin[j] = static_cast<unsigned int>(q0);
				idx_bmax[j] = static_cast<unsigned int>(q1);

				if (idx_bmin[j] >= bin_size)
					idx_bmin[j] = static_cast<unsigned int>(bin_size) - 1;
				if (idx_bmax[j] >= bin_size)
					idx_bmax[j] = static_cast<unsigned int>(bin_size) - 1;

				assert(idx_bmin[j] < bin_size);
				assert(idx_bmax[j] < bin_size);

				// Increment bin counter
				bins->bin[0 * (bins->bin_size * 3) + static_cast<size_t>(j) * bins->bin_size + idx_bmin[j]] += 1;
				bins->bin[1 * (bins->bin_size * 3) + static_cast<size_t>(j) * bins->bin_size + idx_bmax[j]] += 1;
			}
		}
	}

	template<typename T>
	inline T SAH(size_t ns1, T leftArea, size_t ns2, T rightArea, T invS, T Taabb, T Ttri) {
		T sah;

		sah = static_cast<T>(2.0) * Taabb + (leftArea * invS) * static_cast<T>(ns1) * Ttri + (rightArea * invS) * static_cast<T>(ns2) * Ttri;

		return sah;
	}

	template<typename T>
	inline bool FindCutFromBinBuffer(T *cut_pos,		   // [out] xyz
									 int *minCostAxis,	   // [out]
									 const BinBuffer *bins, const real3<T> &bmin, const real3<T> &bmax, size_t num_primitives,
									 T costTaabb) {		   // should be in [0.0, 1.0]
		const T kEPS = std::numeric_limits<T>::epsilon();  // * epsScale;

		size_t left, right;
		real3<T> bsize, bstep;
		real3<T> bminLeft, bmaxLeft;
		real3<T> bminRight, bmaxRight;
		T saLeft, saRight, saTotal;
		T pos;
		T minCost[3];

		T costTtri = 1.0f - costTaabb;

		(*minCostAxis) = 0;

		bsize = bmax - bmin;
		bstep = bsize * (1.0f / bins->bin_size);
		saTotal = CalculateSurfaceArea(bmin, bmax);

		T invSaTotal = 0.0f;
		if (saTotal > kEPS) {
			invSaTotal = 1.0f / saTotal;
		}

		for (int j = 0; j < 3; ++j) {
			//
			// Compute SAH cost for right side of each cell of the bbox.
			// Exclude both extreme side of the bbox.
			//
			//  i:      0    1    2    3
			//     +----+----+----+----+----+
			//     |    |    |    |    |    |
			//     +----+----+----+----+----+
			//

			T minCostPos = bmin[j] + 0.5f * bstep[j];
			minCost[j] = std::numeric_limits<T>::max();

			left = 0;
			right = num_primitives;
			bminLeft = bminRight = bmin;
			bmaxLeft = bmaxRight = bmax;

			for (int i = 0; i < static_cast<int>(bins->bin_size) - 1; ++i) {
				left += bins->bin[0 * (3 * bins->bin_size) + static_cast<size_t>(j) * bins->bin_size + static_cast<size_t>(i)];
				right -= bins->bin[1 * (3 * bins->bin_size) + static_cast<size_t>(j) * bins->bin_size + static_cast<size_t>(i)];

				assert(left <= num_primitives);
				assert(right <= num_primitives);

				//
				// Split pos bmin + (i + 1) * (bsize / BIN_SIZE)
				// +1 for i since we want a position on right side of the cell.
				//

				pos = bmin[j] + (i + 0.5f) * bstep[j];
				bmaxLeft[j] = pos;
				bminRight[j] = pos;

				saLeft = CalculateSurfaceArea(bminLeft, bmaxLeft);
				saRight = CalculateSurfaceArea(bminRight, bmaxRight);

				T cost = SAH(left, saLeft, right, saRight, invSaTotal, costTaabb, costTtri);
				if (cost < minCost[j]) {
					//
					// Update the min cost
					//
					minCost[j] = cost;
					minCostPos = pos;
					// minCostAxis = j;
				}
			}

			cut_pos[j] = minCostPos;
		}

		// cut_axis = minCostAxis;
		// cut_pos = minCostPos;

		// Find min cost axis
		T cost = minCost[0];
		(*minCostAxis) = 0;
		if (cost > minCost[1]) {
			(*minCostAxis) = 1;
			cost = minCost[1];
		}
		if (cost > minCost[2]) {
			(*minCostAxis) = 2;
			cost = minCost[2];
		}

		return true;
	}

#ifdef _OPENMP
	template<typename T, class P>
	void ComputeBoundingBoxOMP(real3<T> *bmin, real3<T> *bmax, const unsigned int *indices, unsigned int left_index, unsigned int right_index,
							   const P &p) {
		{
			p.BoundingBox(bmin, bmax, indices[left_index]);
		}

		T local_bmin[3] = { (*bmin)[0], (*bmin)[1], (*bmin)[2] };
		T local_bmax[3] = { (*bmax)[0], (*bmax)[1], (*bmax)[2] };

		unsigned int n = right_index - left_index;

#pragma omp parallel firstprivate(local_bmin, local_bmax) if (n > (1024 * 128))
		{
#pragma omp for
			for (size_t i = left_index; i < right_index; i++) {	 // for each faces
				unsigned int idx = indices[i];

				real3<T> bbox_min, bbox_max;
				p.BoundingBox(&bbox_min, &bbox_max, idx);
				for (int k = 0; k < 3; k++) {  // xyz
					if ((*bmin)[k] > bbox_min[k])
						(*bmin)[k] = bbox_min[k];
					if ((*bmax)[k] < bbox_max[k])
						(*bmax)[k] = bbox_max[k];
				}
			}

#pragma omp critical
			{
				for (int k = 0; k < 3; k++) {
					if (local_bmin[k] < (*bmin)[k]) {
						{
							if (local_bmin[k] < (*bmin)[k])
								(*bmin)[k] = local_bmin[k];
						}
					}

					if (local_bmax[k] > (*bmax)[k]) {
						{
							if (local_bmax[k] > (*bmax)[k])
								(*bmax)[k] = local_bmax[k];
						}
					}
				}
			}
		}
	}
#endif

	template<typename T, class P>
	inline void ComputeBoundingBox(real3<T> *bmin, real3<T> *bmax, const unsigned int *indices, unsigned int left_index, unsigned int right_index,
								   const P &p) {
		{
			unsigned int idx = indices[left_index];
			p.BoundingBox(bmin, bmax, idx);
		}

		{
			for (unsigned int i = left_index + 1; i < right_index; i++) {  // for each primitives
				unsigned int idx = indices[i];
				real3<T> bbox_min, bbox_max;
				p.BoundingBox(&bbox_min, &bbox_max, idx);
				for (int k = 0; k < 3; k++) {  // xyz
					if ((*bmin)[k] > bbox_min[k])
						(*bmin)[k] = bbox_min[k];
					if ((*bmax)[k] < bbox_max[k])
						(*bmax)[k] = bbox_max[k];
				}
			}
		}
	}

	template<typename T>
	inline void GetBoundingBox(real3<T> *bmin, real3<T> *bmax, const std::vector<BBox<T> > &bboxes, unsigned int *indices, unsigned int left_index,
							   unsigned int right_index) {
		{
			unsigned int i = left_index;
			unsigned int idx = indices[i];
			(*bmin)[0] = bboxes[idx].bmin[0];
			(*bmin)[1] = bboxes[idx].bmin[1];
			(*bmin)[2] = bboxes[idx].bmin[2];
			(*bmax)[0] = bboxes[idx].bmax[0];
			(*bmax)[1] = bboxes[idx].bmax[1];
			(*bmax)[2] = bboxes[idx].bmax[2];
		}

		T local_bmin[3] = { (*bmin)[0], (*bmin)[1], (*bmin)[2] };
		T local_bmax[3] = { (*bmax)[0], (*bmax)[1], (*bmax)[2] };

		{
			for (unsigned int i = left_index; i < right_index; i++) {  // for each faces
				unsigned int idx = indices[i];

				for (int k = 0; k < 3; k++) {						   // xyz
					T minval = bboxes[idx].bmin[k];
					T maxval = bboxes[idx].bmax[k];
					if (local_bmin[k] > minval)
						local_bmin[k] = minval;
					if (local_bmax[k] < maxval)
						local_bmax[k] = maxval;
				}
			}

			for (int k = 0; k < 3; k++) {
				(*bmin)[k] = local_bmin[k];
				(*bmax)[k] = local_bmax[k];
			}
		}
	}

	//
	// --
	//

#if NANORT_ENABLE_PARALLEL_BUILD
	template<typename T, class P, class Pred, class I>
	unsigned int BVHAccel<T, P, Pred, I>::BuildShallowTree(std::vector<BVHNode<T> > *out_nodes, unsigned int left_idx, unsigned int right_idx,
														   unsigned int depth, unsigned int max_shallow_depth, const P &p, const Pred &pred) {
		assert(left_idx <= right_idx);

		unsigned int offset = static_cast<unsigned int>(out_nodes->size());

		if (stats_.max_tree_depth < depth) {
			stats_.max_tree_depth = depth;
		}

		real3<T> bmin, bmax;
		ComputeBoundingBox(&bmin, &bmax, &indices_.at(0), left_idx, right_idx, p);

		unsigned int n = right_idx - left_idx;
		if ((n < options_.min_leaf_primitives) || (depth >= options_.max_tree_depth)) {
			// Create leaf node.
			BVHNode<T> leaf;

			leaf.bmin[0] = bmin[0];
			leaf.bmin[1] = bmin[1];
			leaf.bmin[2] = bmin[2];

			leaf.bmax[0] = bmax[0];
			leaf.bmax[1] = bmax[1];
			leaf.bmax[2] = bmax[2];

			assert(left_idx < std::numeric_limits<unsigned int>::max());

			leaf.flag = 1;	// leaf
			leaf.data[0] = n;
			leaf.data[1] = left_idx;

			out_nodes->push_back(leaf);	 // atomic update

			stats_.num_leaf_nodes++;

			return offset;
		}

		//
		// Create branch node.
		//
		if (depth >= max_shallow_depth) {
			// Delay to build tree
			ShallowNodeInfo info;
			info.left_idx = left_idx;
			info.right_idx = right_idx;
			info.offset = offset;
			shallow_node_infos_.push_back(info);

			// Add dummy node.
			BVHNode<T> node;
			node.axis = -1;
			node.flag = -1;
			out_nodes->push_back(node);

			return offset;

		} else {
			//
			// Compute SAH and find best split axis and position
			//
			int min_cut_axis = 0;
			T cut_pos[3] = { 0.0, 0.0, 0.0 };

			BinBuffer bins(options_.bin_size);
			ContributeBinBuffer(&bins, bmin, bmax, &indices_.at(0), left_idx, right_idx, p);
			FindCutFromBinBuffer(cut_pos, &min_cut_axis, &bins, bmin, bmax, n, options_.cost_t_aabb);

			// Try all 3 axis until good cut position avaiable.
			unsigned int mid_idx = left_idx;
			int cut_axis = min_cut_axis;
			for (int axis_try = 0; axis_try < 3; axis_try++) {
				unsigned int *begin = &indices_[left_idx];
				unsigned int *end = &indices_[right_idx - 1] + 1;  // mimics end() iterator.
				unsigned int *mid = 0;

				// try min_cut_axis first.
				cut_axis = (min_cut_axis + axis_try) % 3;

				// @fixme { We want some thing like: std::partition(begin, end,
				// pred(cut_axis, cut_pos[cut_axis])); }
				pred.Set(cut_axis, cut_pos[cut_axis]);
				//
				// Split at (cut_axis, cut_pos)
				// indices_ will be modified.
				//
				mid = std::partition(begin, end, pred);

				mid_idx = left_idx + static_cast<unsigned int>((mid - begin));
				if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
					// Can't split well.
					// Switch to object median(which may create unoptimized tree, but
					// stable)
					mid_idx = left_idx + (n >> 1);

					// Try another axis if there's axis to try.

				} else {
					// Found good cut. exit loop.
					break;
				}
			}

			BVHNode<T> node;
			node.axis = cut_axis;
			node.flag = 0;	// 0 = branch

			out_nodes->push_back(node);

			unsigned int left_child_index = 0;
			unsigned int right_child_index = 0;

			left_child_index = BuildShallowTree(out_nodes, left_idx, mid_idx, depth + 1, max_shallow_depth, p, pred);

			right_child_index = BuildShallowTree(out_nodes, mid_idx, right_idx, depth + 1, max_shallow_depth, p, pred);

			(*out_nodes)[offset].data[0] = left_child_index;
			(*out_nodes)[offset].data[1] = right_child_index;

			(*out_nodes)[offset].bmin[0] = bmin[0];
			(*out_nodes)[offset].bmin[1] = bmin[1];
			(*out_nodes)[offset].bmin[2] = bmin[2];

			(*out_nodes)[offset].bmax[0] = bmax[0];
			(*out_nodes)[offset].bmax[1] = bmax[1];
			(*out_nodes)[offset].bmax[2] = bmax[2];
		}

		stats_.num_branch_nodes++;

		return offset;
	}
#endif

	template<typename T, class P, class Pred, class I>
	unsigned int BVHAccel<T, P, Pred, I>::BuildTree(BVHBuildStatistics *out_stat, std::vector<BVHNode<T> > *out_nodes, unsigned int left_idx,
													unsigned int right_idx, unsigned int depth, const P &p, const Pred &pred) {
		assert(left_idx <= right_idx);

		unsigned int offset = static_cast<unsigned int>(out_nodes->size());

		if (out_stat->max_tree_depth < depth) {
			out_stat->max_tree_depth = depth;
		}

		real3<T> bmin, bmax;
		if (!bboxes_.empty()) {
			GetBoundingBox(&bmin, &bmax, bboxes_, &indices_.at(0), left_idx, right_idx);
		} else {
			ComputeBoundingBox(&bmin, &bmax, &indices_.at(0), left_idx, right_idx, p);
		}

		unsigned int n = right_idx - left_idx;
		if ((n < options_.min_leaf_primitives) || (depth >= options_.max_tree_depth)) {
			// Create leaf node.
			BVHNode<T> leaf;

			leaf.bmin[0] = bmin[0];
			leaf.bmin[1] = bmin[1];
			leaf.bmin[2] = bmin[2];

			leaf.bmax[0] = bmax[0];
			leaf.bmax[1] = bmax[1];
			leaf.bmax[2] = bmax[2];

			assert(left_idx < std::numeric_limits<unsigned int>::max());

			leaf.flag = 1;	// leaf
			leaf.data[0] = n;
			leaf.data[1] = left_idx;

			out_nodes->push_back(leaf);	 // atomic update

			out_stat->num_leaf_nodes++;

			return offset;
		}

		//
		// Create branch node.
		//

		//
		// Compute SAH and find best split axis and position
		//
		int min_cut_axis = 0;
		T cut_pos[3] = { 0.0, 0.0, 0.0 };

		BinBuffer bins(options_.bin_size);
		ContributeBinBuffer(&bins, bmin, bmax, &indices_.at(0), left_idx, right_idx, p);
		FindCutFromBinBuffer(cut_pos, &min_cut_axis, &bins, bmin, bmax, n, options_.cost_t_aabb);

		// Try all 3 axis until good cut position avaiable.
		unsigned int mid_idx = left_idx;
		int cut_axis = min_cut_axis;
		for (int axis_try = 0; axis_try < 3; axis_try++) {
			unsigned int *begin = &indices_[left_idx];
			unsigned int *end = &indices_[right_idx - 1] + 1;  // mimics end() iterator.
			unsigned int *mid = 0;

			// try min_cut_axis first.
			cut_axis = (min_cut_axis + axis_try) % 3;

			pred.Set(cut_axis, cut_pos[cut_axis]);

			//
			// Split at (cut_axis, cut_pos)
			// indices_ will be modified.
			//
			mid = std::partition(begin, end, pred);

			mid_idx = left_idx + static_cast<unsigned int>((mid - begin));
			if ((mid_idx == left_idx) || (mid_idx == right_idx)) {
				// Can't split well.
				// Switch to object median(which may create unoptimized tree, but
				// stable)
				mid_idx = left_idx + (n >> 1);

				// Try another axis to find better cut.

			} else {
				// Found good cut. exit loop.
				break;
			}
		}

		BVHNode<T> node;
		node.axis = cut_axis;
		node.flag = 0;	// 0 = branch

		out_nodes->push_back(node);

		unsigned int left_child_index = 0;
		unsigned int right_child_index = 0;

		left_child_index = BuildTree(out_stat, out_nodes, left_idx, mid_idx, depth + 1, p, pred);

		right_child_index = BuildTree(out_stat, out_nodes, mid_idx, right_idx, depth + 1, p, pred);

		{
			(*out_nodes)[offset].data[0] = left_child_index;
			(*out_nodes)[offset].data[1] = right_child_index;

			(*out_nodes)[offset].bmin[0] = bmin[0];
			(*out_nodes)[offset].bmin[1] = bmin[1];
			(*out_nodes)[offset].bmin[2] = bmin[2];

			(*out_nodes)[offset].bmax[0] = bmax[0];
			(*out_nodes)[offset].bmax[1] = bmax[1];
			(*out_nodes)[offset].bmax[2] = bmax[2];
		}

		out_stat->num_branch_nodes++;

		return offset;
	}

	template<typename T, class P, class Pred, class I>
	bool BVHAccel<T, P, Pred, I>::Build(unsigned int num_primitives, const BVHBuildOptions<T> &options, const P &p, const Pred &pred) {
		options_ = options;
		stats_ = BVHBuildStatistics();

		nodes_.clear();
		bboxes_.clear();

		assert(options_.bin_size > 1);

		unsigned int n = num_primitives;

		//
		// 1. Create triangle indices(this will be permutated in BuildTree)
		//
		indices_.resize(n);

#ifdef _OPENMP
#pragma omp parallel for
#endif
		for (int i = 0; i < static_cast<int>(n); i++) {
			indices_[static_cast<size_t>(i)] = static_cast<unsigned int>(i);
		}

		//
		// 2. Compute bounding box(optional).
		//
		real3<T> bmin, bmax;
		if (options.cache_bbox) {
			bmin[0] = bmin[1] = bmin[2] = std::numeric_limits<T>::max();
			bmax[0] = bmax[1] = bmax[2] = -std::numeric_limits<T>::max();

			bboxes_.resize(n);
			for (size_t i = 0; i < n; i++) {  // for each primitived
				unsigned int idx = indices_[i];

				BBox<T> bbox;
				p.BoundingBox(&(bbox.bmin), &(bbox.bmax), static_cast<unsigned int>(i));
				bboxes_[idx] = bbox;

				for (int k = 0; k < 3; k++) {  // xyz
					if (bmin[k] > bbox.bmin[k]) {
						bmin[k] = bbox.bmin[k];
					}
					if (bmax[k] < bbox.bmax[k]) {
						bmax[k] = bbox.bmax[k];
					}
				}
			}

		} else {
#ifdef _OPENMP
			ComputeBoundingBoxOMP(&bmin, &bmax, &indices_.at(0), 0, n, p);
#else
			ComputeBoundingBox(&bmin, &bmax, &indices_.at(0), 0, n, p);
#endif
		}

//
// 3. Build tree
//
#ifdef _OPENMP
#if NANORT_ENABLE_PARALLEL_BUILD

		// Do parallel build for enoughly large dataset.
		if (n > options.min_primitives_for_parallel_build) {
			BuildShallowTree(&nodes_, 0, n, /* root depth */ 0, options.shallow_depth, p, pred);  // [0, n)

			assert(shallow_node_infos_.size() > 0);

			// Build deeper tree in parallel
			std::vector<std::vector<BVHNode<T> > > local_nodes(shallow_node_infos_.size());
			std::vector<BVHBuildStatistics> local_stats(shallow_node_infos_.size());

#pragma omp parallel for
			for (int i = 0; i < static_cast<int>(shallow_node_infos_.size()); i++) {
				unsigned int left_idx = shallow_node_infos_[i].left_idx;
				unsigned int right_idx = shallow_node_infos_[i].right_idx;
				BuildTree(&(local_stats[i]), &(local_nodes[i]), left_idx, right_idx, options.shallow_depth, p, pred);
			}

			// Join local nodes
			for (int i = 0; i < static_cast<int>(local_nodes.size()); i++) {
				assert(!local_nodes[i].empty());
				size_t offset = nodes_.size();

				// Add offset to child index(for branch node).
				for (size_t j = 0; j < local_nodes[i].size(); j++) {
					if (local_nodes[i][j].flag == 0) {	// branch
						local_nodes[i][j].data[0] += offset - 1;
						local_nodes[i][j].data[1] += offset - 1;
					}
				}

				// replace
				nodes_[shallow_node_infos_[i].offset] = local_nodes[i][0];

				// Skip root element of the local node.
				nodes_.insert(nodes_.end(), local_nodes[i].begin() + 1, local_nodes[i].end());
			}

			// Join statistics
			for (int i = 0; i < static_cast<int>(local_nodes.size()); i++) {
				stats_.max_tree_depth = std::max(stats_.max_tree_depth, local_stats[i].max_tree_depth);
				stats_.num_leaf_nodes += local_stats[i].num_leaf_nodes;
				stats_.num_branch_nodes += local_stats[i].num_branch_nodes;
			}

		} else {
			BuildTree(&stats_, &nodes_, 0, n,
					  /* root depth */ 0, p, pred);	 // [0, n)
		}

#else												 // !NANORT_ENABLE_PARALLEL_BUILD
		{
			BuildTree(&stats_, &nodes_, 0, n,
					  /* root depth */ 0, p, pred);	 // [0, n)
		}
#endif
#else												 // !_OPENMP
		{
			BuildTree(&stats_, &nodes_, 0, n,
					  /* root depth */ 0, p, pred);	 // [0, n)
		}
#endif

		return true;
	}

	template<typename T, class P, class Pred, class I>
	bool BVHAccel<T, P, Pred, I>::Dump(const char *filename) {
		FILE *fp = fopen(filename, "wb");
		if (!fp) {
			fprintf(stderr, "[BVHAccel] Cannot write a file: %s\n", filename);
			return false;
		}

		size_t numNodes = nodes_.size();
		assert(nodes_.size() > 0);

		size_t numIndices = indices_.size();

		size_t r = 0;
		r = fwrite(&numNodes, sizeof(size_t), 1, fp);
		assert(r == 1);

		r = fwrite(&nodes_.at(0), sizeof(BVHNode<T>), numNodes, fp);
		assert(r == numNodes);

		r = fwrite(&numIndices, sizeof(size_t), 1, fp);
		assert(r == 1);

		r = fwrite(&indices_.at(0), sizeof(unsigned int), numIndices, fp);
		assert(r == numIndices);

		fclose(fp);

		return true;
	}

	template<typename T, class P, class Pred, class I>
	bool BVHAccel<T, P, Pred, I>::Load(const char *filename) {
		FILE *fp = fopen(filename, "rb");
		if (!fp) {
			fprintf(stderr, "Cannot open file: %s\n", filename);
			return false;
		}

		size_t numNodes;
		size_t numIndices;

		size_t r = 0;
		r = fread(&numNodes, sizeof(size_t), 1, fp);
		assert(r == 1);
		assert(numNodes > 0);

		nodes_.resize(numNodes);
		r = fread(&nodes_.at(0), sizeof(BVHNode<T>), numNodes, fp);
		assert(r == numNodes);

		r = fread(&numIndices, sizeof(size_t), 1, fp);
		assert(r == 1);

		indices_.resize(numIndices);

		r = fread(&indices_.at(0), sizeof(unsigned int), numIndices, fp);
		assert(r == numIndices);

		fclose(fp);

		return true;
	}

	template<typename T>
	inline bool IntersectRayAABB(T *tminOut,  // [out]
								 T *tmaxOut,  // [out]
								 T min_t, T max_t, const T bmin[3], const T bmax[3], real3<T> ray_org, real3<T> ray_inv_dir, int ray_dir_sign[3]) {
		T tmin, tmax;

		const T min_x = ray_dir_sign[0] ? bmax[0] : bmin[0];
		const T min_y = ray_dir_sign[1] ? bmax[1] : bmin[1];
		const T min_z = ray_dir_sign[2] ? bmax[2] : bmin[2];
		const T max_x = ray_dir_sign[0] ? bmin[0] : bmax[0];
		const T max_y = ray_dir_sign[1] ? bmin[1] : bmax[1];
		const T max_z = ray_dir_sign[2] ? bmin[2] : bmax[2];

		// X
		const T tmin_x = (min_x - ray_org[0]) * ray_inv_dir[0];
		// MaxMult robust BVH traversal(up to 4 ulp).
		// 1.0000000000000004 for double precision.
		const T tmax_x = (max_x - ray_org[0]) * ray_inv_dir[0] * 1.00000024f;

		// Y
		const T tmin_y = (min_y - ray_org[1]) * ray_inv_dir[1];
		const T tmax_y = (max_y - ray_org[1]) * ray_inv_dir[1] * 1.00000024f;

		// Z
		const T tmin_z = (min_z - ray_org[2]) * ray_inv_dir[2];
		const T tmax_z = (max_z - ray_org[2]) * ray_inv_dir[2] * 1.00000024f;

		tmin = safemax(tmin_z, safemax(tmin_y, safemax(tmin_x, min_t)));
		tmax = safemin(tmax_z, safemin(tmax_y, safemin(tmax_x, max_t)));

		if (tmin <= tmax) {
			(*tminOut) = tmin;
			(*tmaxOut) = tmax;

			return true;
		}

		return false;  // no hit
	}

	template<typename T, class P, class Pred, class I>
	inline bool BVHAccel<T, P, Pred, I>::TestLeafNode(const BVHNode<T> &node, const Ray<T> &ray, const I &intersector) const {
		bool hit = false;

		unsigned int num_primitives = node.data[0];
		unsigned int offset = node.data[1];

		T t = intersector.GetT();  // current hit distance

		real3<T> ray_org;
		ray_org[0] = ray.org[0];
		ray_org[1] = ray.org[1];
		ray_org[2] = ray.org[2];

		real3<T> ray_dir;
		ray_dir[0] = ray.dir[0];
		ray_dir[1] = ray.dir[1];
		ray_dir[2] = ray.dir[2];

		for (unsigned int i = 0; i < num_primitives; i++) {
			unsigned int prim_idx = indices_[i + offset];

			T local_t = t;
			if (intersector.Intersect(&local_t, prim_idx)) {
				if (local_t > ray.min_t) {
					// Update isect state
					t = local_t;

					intersector.Update(t, prim_idx);
					hit = true;
				}
			}
		}

		return hit;
	}

#if 0
template <typename T, class P, class Pred, class I>
bool BVHAccel<T, P, Pred, I>::MultiHitTestLeafNode(const BVHNode &node,
  const Ray &ray, int max_intersections, const I &intersector) const {
  bool hit = false;

  unsigned int num_primitives = node.data[0];
  unsigned int offset = node.data[1];

  T t = std::numeric_limits<T>::max();
  if (isects->size() >= static_cast<size_t>(max_intersections)) {
    t = isects->top().t;  // current furthest hit distance
  }

  real3 ray_org;
  ray_org[0] = ray.org[0];
  ray_org[1] = ray.org[1];
  ray_org[2] = ray.org[2];

  real3 ray_dir;
  ray_dir[0] = ray.dir[0];
  ray_dir[1] = ray.dir[1];
  ray_dir[2] = ray.dir[2];

  for (unsigned int i = 0; i < num_primitives; i++) {
    unsigned int prim_idx = indices_[i + offset];

    T local_t = t, u = 0.0f, v = 0.0f;
    if (p.Intersect(&local_t, &u, &v, prim_idx)) {
      // Update isect state
      if ((local_t > ray.min_t)) {
        if (isects->size() < static_cast<size_t>(max_intersections)) {
          Intersection isect;
          t = local_t;
          isect.t = t;
          isect.u = u;
          isect.v = v;
          isect.prim_id = prim_idx;
          isects->push(isect);

          // Update t to furthest distance.
          t = ray.max_t;

          hit = true;
        } else {
          if (local_t < isects->top().t) {
            // delete furthest intersection and add new intersection.
            isects->pop();

            Intersection isect;
            isect.t = local_t;
            isect.u = u;
            isect.v = v;
            isect.prim_id = prim_idx;
            isects->push(isect);

            // Update furthest hit distance
            t = isects->top().t;

            hit = true;
          }
        }
      }
    }
  }

  return hit;
}
#endif

	template<typename T, class P, class Pred, class I>
	bool BVHAccel<T, P, Pred, I>::Traverse(const Ray<T> &ray, const BVHTraceOptions &options, const I &intersector) const {
		const int kMaxStackDepth = 512;

		T hit_t = ray.max_t;

		int node_stack_index = 0;
		unsigned int node_stack[512];
		node_stack[0] = 0;

		// Init isect info as no hit
		intersector.Update(hit_t, static_cast<unsigned int>(-1));

		intersector.PrepareTraversal(ray, options);

		int dir_sign[3];
		dir_sign[0] = ray.dir[0] < 0.0f ? 1 : 0;
		dir_sign[1] = ray.dir[1] < 0.0f ? 1 : 0;
		dir_sign[2] = ray.dir[2] < 0.0f ? 1 : 0;

		// @fixme { Check edge case; i.e., 1/0 }
		real3<T> ray_inv_dir;
		ray_inv_dir[0] = 1.0f / (ray.dir[0] + 1.0e-12f);
		ray_inv_dir[1] = 1.0f / (ray.dir[1] + 1.0e-12f);
		ray_inv_dir[2] = 1.0f / (ray.dir[2] + 1.0e-12f);

		real3<T> ray_org;
		ray_org[0] = ray.org[0];
		ray_org[1] = ray.org[1];
		ray_org[2] = ray.org[2];

		T min_t = std::numeric_limits<T>::max();
		T max_t = -std::numeric_limits<T>::max();

		while (node_stack_index >= 0) {
			unsigned int index = node_stack[node_stack_index];
			const BVHNode<T> &node = nodes_[index];

			node_stack_index--;

			bool hit = IntersectRayAABB(&min_t, &max_t, ray.min_t, hit_t, node.bmin, node.bmax, ray_org, ray_inv_dir, dir_sign);

			if (node.flag == 0) {  // branch node
				if (hit) {
					int order_near = dir_sign[node.axis];
					int order_far = 1 - order_near;

					// Traverse near first.
					node_stack[++node_stack_index] = node.data[order_far];
					node_stack[++node_stack_index] = node.data[order_near];
				}
			} else {  // leaf node
				if (hit) {
					if (TestLeafNode(node, ray, intersector)) {
						hit_t = intersector.GetT();
					}
				}
			}
		}

		assert(node_stack_index < kMaxStackDepth);

		bool hit = (intersector.GetT() < ray.max_t);
		intersector.PostTraversal(ray, hit);

		return hit;
	}

#if 0
template <typename T, class P, class Pred, class I>
bool BVHAccel<T, P, Pred, I>::MultiHitTraverse(const Ray &ray,
                                         const BVHTraceOptions &options,
                                         int max_intersections,
                                         StackVector<I, 128> *isects) const {
  const int kMaxStackDepth = 512;

  T hit_t = ray.max_t;

  int node_stack_index = 0;
  unsigned int node_stack[512];
  node_stack[0] = 0;

  // Stores furthest intersection at top
  std::priority_queue<I, std::vector<I>, IsectComparator<I> >  isect_pq;
  //// Stores furthest intersection at top
  // template<class I>
  // typedef std::priority_queue<I, std::vector<I>,
  //                            IsectComparator<I> >
  //    IsectVector;

  (*isects)->clear();

  p.PrepareTraversal(ray, options);

  int dir_sign[3];
  dir_sign[0] = ray.dir[0] < 0.0f ? 1 : 0;
  dir_sign[1] = ray.dir[1] < 0.0f ? 1 : 0;
  dir_sign[2] = ray.dir[2] < 0.0f ? 1 : 0;

  // @fixme { Check edge case; i.e., 1/0 }
  real3 ray_inv_dir;
  ray_inv_dir[0] = 1.0f / ray.dir[0];
  ray_inv_dir[1] = 1.0f / ray.dir[1];
  ray_inv_dir[2] = 1.0f / ray.dir[2];

  real3 ray_org;
  ray_org[0] = ray.org[0];
  ray_org[1] = ray.org[1];
  ray_org[2] = ray.org[2];

  T min_t, max_t;
  while (node_stack_index >= 0) {
    unsigned int index = node_stack[node_stack_index];
    const BVHNode &node = nodes_[static_cast<size_t>(index)];

    node_stack_index--;

    bool hit = IntersectRayAABB(&min_t, &max_t, ray.min_t, hit_t, node.bmin,
                                node.bmax, ray_org, ray_inv_dir, dir_sign);

    if (node.flag == 0) {  // branch node
      if (hit) {
        int order_near = dir_sign[node.axis];
        int order_far = 1 - order_near;

        // Traverse near first.
        node_stack[++node_stack_index] = node.data[order_far];
        node_stack[++node_stack_index] = node.data[order_near];
      }

    } else {  // leaf node
      if (hit) {
        if (MultiHitTestLeafNode(&isect_pq, max_intersections, node, ray, p)) {
          // Only update `hit_t` when queue is full.
          if (isect_pq.size() >= static_cast<size_t>(max_intersections)) {
            hit_t = isect_pq.top().t;
          }
        }
      }
    }
  }

  assert(node_stack_index < kMaxStackDepth);

  if (!isect_pq.empty()) {
    // Store intesection in reverse order(make it frontmost order)
    size_t n = isect_pq.size();
    (*isects)->resize(n);
    for (size_t i = 0; i < n; i++) {
      const Intersection &isect = isect_pq.top();
      (*isects)[n - i - 1] = isect;
      isect_pq.pop();
    }

    return true;
  }

  return false;
}
#endif

}  // namespace nanort

#endif	// NANORT_H_
