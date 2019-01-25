#ifndef CUSTOM_ROS_DEBUG_H_
#define CUSTOM_ROS_DEBUG_H_

namespace pc
{
    enum PRINT_COLOR
    {
        BLACK,
        RED,
        GREEN,
        YELLOW,
        BLUE,
        MAGENTA,
        CYAN,
        WHITE,
        ENDCOLOR
    };

    std::ostream& operator<<(std::ostream& os, PRINT_COLOR c)
    {
        switch(c)
        {
            case BLACK    : os << "\033[1;30m"; break;
            case RED      : os << "\033[1;31m"; break;
            case GREEN    : os << "\033[1;32m"; break;
            case YELLOW   : os << "\033[1;33m"; break;
            case BLUE     : os << "\033[1;34m"; break;
            case MAGENTA  : os << "\033[1;35m"; break;
            case CYAN     : os << "\033[1;36m"; break;
            case WHITE    : os << "\033[1;37m"; break;
            case ENDCOLOR : os << "\033[0m";    break;
            default       : os << "\033[1;37m";
        }
        return os;
    }
} //namespace pc

#define ROS_BLACK_STREAM(x)   ROS_INFO_STREAM(pc::BLACK   << x << pc::ENDCOLOR)
#define ROS_RED_STREAM(x)     ROS_INFO_STREAM(pc::RED     << x << pc::ENDCOLOR)
#define ROS_GREEN_STREAM(x)   ROS_INFO_STREAM(pc::GREEN   << x << pc::ENDCOLOR)
#define ROS_YELLOW_STREAM(x)  ROS_INFO_STREAM(pc::YELLOW  << x << pc::ENDCOLOR)
#define ROS_BLUE_STREAM(x)    ROS_INFO_STREAM(pc::BLUE    << x << pc::ENDCOLOR)
#define ROS_MAGENTA_STREAM(x) ROS_INFO_STREAM(pc::MAGENTA << x << pc::ENDCOLOR)
#define ROS_CYAN_STREAM(x)    ROS_INFO_STREAM(pc::CYAN    << x << pc::ENDCOLOR)

#define ROS_BLACK_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, pc::BLACK   << x << pc::ENDCOLOR)
#define ROS_RED_STREAM_COND(c, x)     ROS_INFO_STREAM_COND(c, pc::RED     << x << pc::ENDCOLOR)
#define ROS_GREEN_STREAM_COND(c, x)   ROS_INFO_STREAM_COND(c, pc::GREEN   << x << pc::ENDCOLOR)
#define ROS_YELLOW_STREAM_COND(c, x)  ROS_INFO_STREAM_COND(c, pc::YELLOW  << x << pc::ENDCOLOR)
#define ROS_BLUE_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, pc::BLUE    << x << pc::ENDCOLOR)
#define ROS_MAGENTA_STREAM_COND(c, x) ROS_INFO_STREAM_COND(c, pc::MAGENTA << x << pc::ENDCOLOR)
#define ROS_CYAN_STREAM_COND(c, x)    ROS_INFO_STREAM_COND(c, pc::CYAN    << x << pc::ENDCOLOR)

#endif /** CUSTOM_ROS_DEBUG_H_ */
