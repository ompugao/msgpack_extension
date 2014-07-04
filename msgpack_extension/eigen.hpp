#ifndef __MSGPACK_EXTENSION_EIGEN__
#define __MSGPACK_EXTENSION_EIGEN__

#include "vector.hpp"

namespace msgpack {
/*{{{*/
/*
inline void msgpack_unpack(msgpack::object o) {
    if(o.type != msgpack::type::ARRAY) { throw msgpack::type_error(); }

    msgpack::object * p = o.via.array.ptr;

    std::string type;
    *p >> type;
    if (type != "__eigen__") { throw msgpack::type_error(); }

    size_t rows;
    size_t cols;

    ++p;
    *p >> rows;
    ++p;
    *p >> cols;
    this->resize(rows, cols);

    for (size_t i = 0; i < this->rows(); ++i) {
        for (size_t j = 0; j < this->cols(); ++j) {
            ++p;
            *p >> this->operator()(i, j);
        }
    }
}

template <typename Packer>
inline void msgpack_pack(Packer& pk) const {
    pk.pack_array(3 + this->rows()*this->cols());
    pk.pack(std::string("__eigen__"));
    pk.pack(this->rows());
    pk.pack(this->cols());

    for (size_t i = 0; i < this->rows(); ++i) {
        for (size_t j = 0; j < this->cols(); ++j) {
            pk.pack(this->operator()(i, j));
        }
    }
}

template <typename MSGPACK_OBJECT>
inline void msgpack_object(MSGPACK_OBJECT* o, msgpack::zone* z) const { }
*/
/*}}}*/
template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols > & operator>> (object o, Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	if(o.via.array.size > 0) {
        msgpack::object * p = o.via.array.ptr;
        size_t rows;
        size_t cols;
        *p >> rows; ++p;
        *p >> cols;
        v.resize(rows, cols);

        for (int i = 0; i < v.rows(); ++i) {
            for (int j = 0; j < v.cols(); ++j) {
                ++p;
                *p >> v.operator()(i, j);
            }
        }
	}
	return v;
}

template <typename Stream,typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline packer<Stream>& operator<< (packer<Stream>& o, const Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >& v)
{
    o.pack_array(2 + v.rows()*v.cols());
    o.pack(v.rows());
    o.pack(v.cols());

    for (size_t i = 0; i < v.rows(); ++i) {
        for (size_t j = 0; j < v.cols(); ++j) {
            o.pack(v.operator()(i, j));
        }
    }
	return o;
}

template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline void operator<< (object::with_zone& o, const Eigen::Matrix< _Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols >& v)
{
	o.type = type::ARRAY;
    size_t msgsize = 2 + v.rows()*v.cols();
    object* p = (object*)o.zone->malloc(sizeof(object)*msgsize);
    o.via.array.ptr = p;
    o.via.array.size = msgsize;
    for (size_t i = 0; i < v.rows(); ++i) {
        for (size_t j = 0; j < v.cols(); ++j) {
            *p = object(v.operator()(i, j), o.zone);
            ++p;
        }
    }
}


template<typename Scalar, int Options>
inline Eigen::Quaternion< Scalar, Options > & operator>> (object o, Eigen::Quaternion< Scalar, Options >& v)
{
	if(o.type != type::ARRAY) { throw type_error(); }
	if(o.via.array.size > 0) {
        msgpack::object * p = o.via.array.ptr;

        *p >> v.w();++p;
        *p >> v.x();++p;
        *p >> v.y();++p;
        *p >> v.z();
	}
	return v;
}

template<typename Stream, typename Scalar, int Options>
inline packer<Stream>& operator<< (packer<Stream>& o, const Eigen::Quaternion< Scalar, Options >& v)
{
    o.pack_array(4);
    o.pack(v.w());
    o.pack(v.x());
    o.pack(v.y());
    o.pack(v.z());
	return o;
}

template<typename Scalar, int Options>
inline void operator<< (object::with_zone& o, const Eigen::Quaternion< Scalar, Options >& v)
{
	o.type = type::ARRAY;
    size_t msgsize = 4;
    object* p = (object*)o.zone->malloc(sizeof(object)*msgsize);
    o.via.array.ptr = p;
    o.via.array.size = msgsize;
    *p = object(v.w(), o.zone); ++p;
    *p = object(v.x(), o.zone); ++p;
    *p = object(v.y(), o.zone); ++p;
    *p = object(v.z(), o.zone);
}
} // namespace msgpack
#endif /* end of include guard */
