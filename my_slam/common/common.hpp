#ifndef __COMMON_H
#define __COMMON_H

#include <vector>

namespace my_slam
{
    typedef unsigned char pic_byte;
    class picture
    {
    public:
        pic_byte* data;
        short cols;
        short rows;
        picture(pic_byte* _data,short _cols,short _rows)
        {
            data = _data;
            cols = _cols;
            rows = _rows;
        }
        picture() = default;
        ~picture(){}
    };
    typedef std::vector<picture> ImgPyr;
    
    
};


#endif // __COMMON_H
