#ifndef OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_MACRO_DEFINE_H_
#define OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_MACRO_DEFINE_H_

namespace OpticalFlow_SLAM_algorithm_opticalflow_slam {

//the following are UBUNTU/LINUX ONLY terminal color codes.
#define RESET_COLOR   "\033[0m"
#define BLACK   "\033[30m"      /* Black */
#define RED     "\033[31m"      /* Red */
#define GREEN   "\033[32m"      /* Green */
#define YELLOW  "\033[33m"      /* Yellow */
#define BLUE    "\033[34m"      /* Blue */
#define MAGENTA "\033[35m"      /* Magenta */
#define CYAN    "\033[36m"      /* Cyan */
#define WHITE   "\033[37m"      /* White */
#define BOLDBLACK   "\033[1m\033[30m"      /* Bold Black */
#define BOLDRED     "\033[1m\033[31m"      /* Bold Red */
#define BOLDGREEN   "\033[1m\033[32m"      /* Bold Green */
#define BOLDYELLOW  "\033[1m\033[33m"      /* Bold Yellow */
#define BOLDBLUE    "\033[1m\033[34m"      /* Bold Blue */
#define BOLDMAGENTA "\033[1m\033[35m"      /* Bold Magenta */
#define BOLDCYAN    "\033[1m\033[36m"      /* Bold Cyan */
#define BOLDWHITE   "\033[1m\033[37m"      /* Bold White */

//Macro swithc : to print function info or not;
//  #define DEBUG_WITH_FUNCTION_INFO   
#ifdef DEBUG_WITH_FUNCTION_INFO
#define SHOW_FUNCTION_INFO \
        { std::cout <<BOLDCYAN << " INFO :  " << RESET_COLOR << "function: " <<  __FUNCTION__  << "  line: " <<  __LINE__  << "  file: " << __FILE__  << std::endl; }
#else
#define SHOW_FUNCTION_INFO
#endif

#include<glog/logging.h>
#define DLOG_INFO (DLOG(INFO)<< BOLDCYAN<< " D->INFO: "<<RESET_COLOR)
#define DLOG_WARNING (DLOG(WARNING)<< BOLDYELLOW<< " D->WARNING: "<<RESET_COLOR)
#define DLOG_ERROR (DLOG(ERROR)<< BOLDRED<< " D->ERROR: "<<RESET_COLOR) 
#define DLOG_FATAL (DLOG(FATAL)<< BOLDMAGENTA<< " D->FATAL"<<RESET_COLOR) 

#define LOG_INFO (LOG(INFO)<< BOLDCYAN<< " INFO: "<<RESET_COLOR)
#define LOG_WARNING (LOG(WARNING)<< BOLDYELLOW<< " WARNING: "<<RESET_COLOR)
#define LOG_ERROR (LOG(ERROR)<< BOLDRED<< " ERROR: "<<RESET_COLOR) 
#define LOG_FATAL (LOG(FATAL)<< BOLDMAGENTA<< " FATAL: "<<RESET_COLOR) 

} //namespace OpticalFlow_SLAM_algorithm_opticalflow_slam

#endif //OPTICALFLOW_SLAM_ALGORITHM_OPTICALFLOW_SLAM_MACRO_DEFINE_H_