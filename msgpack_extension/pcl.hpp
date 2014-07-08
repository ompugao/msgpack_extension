#ifndef __MSGPACK_EXTENSION_PCL__
#define __MSGPACK_EXTENSION_PCL__

#include <msgpack.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "eigen.hpp"
#include <iostream>

namespace msgpack {


/************** PointXYZ *****************/
inline pcl::PointXYZ& operator>> (object o, pcl::PointXYZ& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	if(o.via.array.size > 0) {
		object* p = o.via.array.ptr;
		object* const pend = o.via.array.ptr + o.via.array.size;
		float* it = &v.data[0];
		do {
			p->convert(it);
			++p;
			++it;
		} while(p < pend);
	}
	return v;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, const pcl::PointXYZ& v)
{
	o.pack_array(4);
    o.pack(v.data[0]);
    o.pack(v.data[1]);
    o.pack(v.data[2]);
    o.pack(v.data[3]);
	return o;
}

inline void operator<< (object::with_zone& o, const pcl::PointXYZ& v)
{
	o.type = type::ARRAY;
    object* p = (object*)o.zone->malloc(sizeof(object)*4);
    object* const pend = p + 4;
    o.via.array.ptr = p;
    o.via.array.size = 4;
    size_t i=0;
    do {
        *p = object(v.data[i], o.zone);
        ++p;
        ++i;
    } while(p < pend);
}
/************** end of PointXYZ *****************/

/************** Normal *****************/
inline pcl::Normal& operator>> (object o, pcl::Normal& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	if(o.via.array.size > 0) {
        if(o.via.array.size!=4) { throw type_error(); }
		object* p = o.via.array.ptr;
		object* const pend = o.via.array.ptr + 4;
		float* it = &v.data_n[0];
		do {
			p->convert(it);
			++p;
			++it;
		} while(p < pend);
	}
	return v;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, const pcl::Normal& v)
{
	o.pack_array(4);
    o.pack(v.data_n[0]);
    o.pack(v.data_n[1]);
    o.pack(v.data_n[2]);
    o.pack(v.data_n[3]);
	return o;
}

inline void operator<< (object::with_zone& o, const pcl::Normal& v)
{
	o.type = type::ARRAY;
    object* p = (object*)o.zone->malloc(sizeof(object)*4);
    object* const pend = p + 4;
    o.via.array.ptr = p;
    o.via.array.size = 4;
    size_t i=0;
    do {
        *p = object(v.data_n[i], o.zone);
        ++p;
        ++i;
    } while(p < pend);
}
/************** end of Normal *****************/

/************** PointNormal *****************/
inline pcl::PointNormal& operator>> (object o, pcl::PointNormal& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	if(o.via.array.size > 0) {
        if(o.via.array.size!=12) { throw type_error(); }
		object* p = o.via.array.ptr;
		object* const pend = o.via.array.ptr + 4;
		float* it = &v.data[0];
		do {
			p->convert(it);
			++p;
			++it;
		} while(p < pend);

		it = &v.data_n[0];
        object* const pend2 = o.via.array.ptr + 8;
		do {
			p->convert(it);
			++p;
			++it;
		} while(p < pend2);

		it = &v.data_c[0];
        object* const pend3 = o.via.array.ptr + 12;
		do {
			p->convert(it);
			++p;
			++it;
		} while(p < pend3);
	}
	return v;
}

template <typename Stream>
inline packer<Stream>& operator<< (packer<Stream>& o, const pcl::PointNormal& v)
{
	o.pack_array(12);
    o.pack(v.data[0]);
    o.pack(v.data[1]);
    o.pack(v.data[2]);
    o.pack(v.data[3]);
    o.pack(v.data_n[0]);
    o.pack(v.data_n[1]);
    o.pack(v.data_n[2]);
    o.pack(v.data_n[3]);
    o.pack(v.data_c[0]);
    o.pack(v.data_c[1]);
    o.pack(v.data_c[2]);
    o.pack(v.data_c[3]);
	return o;
}

inline void operator<< (object::with_zone& o, const pcl::PointNormal& v)
{
	o.type = type::ARRAY;
    object* p = (object*)o.zone->malloc(sizeof(object)*12);
    object* const pend = p + 4;
    object* const pend2 = p + 8;
    object* const pend3 = p + 12;
    o.via.array.ptr = p;
    o.via.array.size = 12;
    size_t i = 0;
    do {
        *p = object(v.data[i], o.zone);
        ++p;
        ++i;
    } while(p < pend);
    do {
        *p = object(v.data_n[i], o.zone);
        ++p;
        ++i;
    } while(p < pend2);
    i=0;
    do {
        *p = object(v.data_c[i], o.zone);
        ++p;
        ++i;
    } while(p < pend3);
}
/************** end of PointNormal *****************/

/************** PointCloud<T> *****************/

template <typename T>
inline pcl::PointCloud<T>& operator>> (object o, pcl::PointCloud<T>& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	if(o.via.array.size > 0) {
		object* p = o.via.array.ptr;
        *p >> v.width;++p;
        *p >> v.height;++p;
        *p >> v.points;++p;
        *p >> v.is_dense;++p;
        *p >> v.sensor_origin_;++p;
        *p >> v.sensor_orientation_;++p;
    }
	return v;
}

template <typename Stream, typename T>
inline packer<Stream>& operator<< (packer<Stream>& o, const pcl::PointCloud<T>& v)
{
	o.pack_array(6);
    o.pack(v.width);
    o.pack(v.height);
    o.pack(v.points);
    o.pack(v.is_dense);
    o.pack(v.sensor_origin_);
    o.pack(v.sensor_orientation_);
	return o;
}

template <typename T>
inline void operator<< (object::with_zone& o, const pcl::PointCloud<T>& v)
{
	o.type = type::ARRAY;
	if(v.empty()) {
		o.via.array.ptr = NULL;
		o.via.array.size = 0;
	} else {
		object* p = (object*)o.zone->malloc(sizeof(object)*6);
		o.via.array.ptr = p;
		o.via.array.size = 6;
        *p = object(v.width, o.zone); ++p;
        *p = object(v.height, o.zone); ++p;
        *p = object(v.points, o.zone); ++p;
        *p = object(v.is_dense, o.zone); ++p;
        *p = object(v.sensor_origin_, o.zone); ++p;
        *p = object(v.sensor_orientation_, o.zone); ++p;
	}
}

}  // namespace msgpack

#endif /* end of include guard */

