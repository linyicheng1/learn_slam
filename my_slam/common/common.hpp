#ifndef __COMMON_H
#define __COMMON_H

#include <vector>
#include <ctime>
namespace my_slam
{
#define PI 3.1415926
    typedef unsigned char pic_byte;
    class picture
    {
    public:
        pic_byte* data;
        int cols;
        int rows;
        picture(pic_byte* _data,int _cols,int _rows)
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
