#ifndef SERIALIZATION_LIBRARY
#define SERIALIZATION_LIBRARY

#include "std_include.h"

#include <boost/serialization/split_free.hpp>
#include <boost/serialization/vector.hpp>

using namespace std;
using namespace cv;

BOOST_SERIALIZATION_SPLIT_FREE(Mat)
namespace boost {
    namespace serialization {
        
        /** Serialization support for cv::Mat */
        template<class Archive>
        void save(Archive & ar, const Mat& m, const unsigned int version) {
            size_t elem_size = m.elemSize();
            size_t elem_type = m.type();
            
            ar & m.cols;
            ar & m.rows;
            ar & elem_size;
            ar & elem_type;
            
            const size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }
        
        /** Serialization support for cv::Mat */
        template<class Archive>
        void load(Archive & ar, Mat& m, const unsigned int version) {
            int cols, rows;
            size_t elem_size, elem_type;
            
            ar & cols;
            ar & rows;
            ar & elem_size;
            ar & elem_type;
            
            m.create(rows, cols, elem_type);
            
            size_t data_size = m.cols * m.rows * elem_size;
            ar & boost::serialization::make_array(m.ptr(), data_size);
        }
 
    }
}

#endif
