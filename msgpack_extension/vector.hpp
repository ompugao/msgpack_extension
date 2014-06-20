#ifndef __MSGPACK_EXTENSION_VECTOR__
#define __MSGPACK_EXTENSION_VECTOR__

#include "msgpack/object.hpp"
#include <vector>

namespace msgpack {


template <typename T, typename U>
inline std::vector<T,U>& operator>> (object o, std::vector<T,U>& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	v.resize(o.via.array.size);
	if(o.via.array.size > 0) {
		object* p = o.via.array.ptr;
		object* const pend = o.via.array.ptr + o.via.array.size;
		T* it = &v[0];
		do {
			p->convert(it);
			++p;
			++it;
		} while(p < pend);
	}
	return v;
}

template <typename Stream, typename T, typename U>
inline packer<Stream>& operator<< (packer<Stream>& o, const std::vector<T,U>& v)
{
	o.pack_array(v.size());
	for(typename std::vector<T,U>::const_iterator it(v.begin()), it_end(v.end());
			it != it_end; ++it) {
		o.pack(*it);
	}
	return o;
}

template <typename T, typename U>
inline void operator<< (object::with_zone& o, const std::vector<T,U>& v)
{
	o.type = type::ARRAY;
	if(v.empty()) {
		o.via.array.ptr = NULL;
		o.via.array.size = 0;
	} else {
		object* p = (object*)o.zone->malloc(sizeof(object)*v.size());
		object* const pend = p + v.size();
		o.via.array.ptr = p;
		o.via.array.size = v.size();
		typename std::vector<T,U>::const_iterator it(v.begin());
		do {
			*p = object(*it, o.zone);
			++p;
			++it;
		} while(p < pend);
	}
}


}  // namespace msgpack

#endif /* msgpack/type/vector.hpp */

