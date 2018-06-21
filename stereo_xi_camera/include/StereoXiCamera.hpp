#ifndef __STEREOXICAMERA_H__
#define __STEREOXICAMERA_H__

// =============== C headers. ====================

/* No hearders specified. */

// ============ C++ standard headers. ============

#include <exception>
#include <string>

// =========== System headers. =================

#include <boost/exception/all.hpp>
#include <boost/shared_ptr.hpp>

// ============ Field specific headers. ========

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

// ========== Application headers. ==============

#include "xiApiPlusOcv.hpp"

// ============ Macros. ========================

#define SXC_NULL (0)

#define N_XI_C 2

#define LOOP_CAMERAS_BEGIN \
	for( int loopIdx = 0; loopIdx < N_XI_C; loopIdx++ )\
	{

#define LOOP_CAMERAS_END \
	}

#define LOOP_CAMERAS_REVERSE_BEGIN \
	for( int loopIdx = N_XI_C - 1; loopIdx >= 0; loopIdx-- )\
	{

#define LOOP_CAMERAS_REVERSE_END \
	}

#define EXCEPTION_ARG_OUT_OF_RANGE(v, minV, maxV) \
    {\
        std::stringstream v##_ss;\
        v##_ss << "Argument out of range, " \
               << #v << " = " << v \
               << ", [" << minV << ", " << maxV << "]."\
               << "Value not changed.";\
        BOOST_THROW_EXCEPTION( argument_out_of_range() << ExceptionInfoString(v##_ss.str()) );\
    }

#define CAMERA_EXCEPTION_DESCRIPTION_BUFFER_SIZE (1024)
#define EXCEPTION_CAMERA_API(camEx) \
    {\
        char camEx##_buffer[CAMERA_EXCEPTION_DESCRIPTION_BUFFER_SIZE];\
        std::stringstream camEx##_ss;\
        \
        camEx.GetDescription(camEx##_buffer, CAMERA_EXCEPTION_DESCRIPTION_BUFFER_SIZE);\
        \
        camEx##_ss << "Camera API throws exception. Error number: "\
                   << camEx.GetErrorNumber()\
                   << ", with description \"" << camEx##_buffer << "\"";\
        \
        BOOST_THROW_EXCEPTION( camera_api_exception() << ExceptionInfoString(camEx##_ss.str()) );\
    }

// ============ File-wise or global variables. ===========

/* No variables declared or initialized. */

// ============= Class definitions. ======================

namespace sxc {

struct exception_base        : virtual std::exception, virtual boost::exception { };
struct bad_argument          : virtual exception_base { };
struct argument_out_of_range : virtual bad_argument { };
struct arguemnt_null         : virtual bad_argument { };
struct camera_api_exception  : virtual exception_base { };

typedef boost::error_info<struct tag_info_string, std::string> ExceptionInfoString;

class StereoXiCamera 
{
public:
    StereoXiCamera(std::string &camSN0, std::string &camSN1);
    ~StereoXiCamera();

    void open();
    void start_acquisition(int waitMS = 500);

    void software_trigger(void);
    void get_images(cv::Mat &img0, cv::Mat &img1);

    void stop_acquisition(int waitMS = 500);
    void close();

    void put_sensor_filter_array(int idx, std::string &strFilterArray);

    // Getters and setters.
    void   set_autogain_exposure_priority(double val);
    double get_autogain_exposure_priority(void);

    void   set_autoexposure_top_limit(int tLimit);
    int    get_autoexposure_top_limit(void);

    void   set_total_bandwidth(int b);
    int    get_total_bandwidth(void);
    void   set_bandwidth_margin(int m);
    int    get_bandwidth_margin(void);
    double get_max_frame_rate(void);

protected:
    void prepare_before_opening();
    void open_and_common_settings();
    void setup_camera_common(xiAPIplusCameraOcv& cam);

    cv::Mat get_single_image(int idx);

    int EXPOSURE_MILLISEC(int val);
    
public:
    const double AUTO_GAIN_EXPOSURE_PRIORITY_MAX;
    const double AUTO_GAIN_EXPOSURE_PRIORITY_MIM;
    const int    AUTO_EXPOSURE_TOP_LIMIT_MAX;     // Millisecond.
    const int    AUTO_EXPOSURE_TOP_LIMIT_MIN;     // Millisecond.
    const int    TOTAL_BANDWIDTH_MAX;             // MBit/s.
    const int    TOTAL_BANDWIDTH_MIN;             // MBit/s.
    const int    BANDWIDTH_MARGIN_MAX;            // %.
    const int    BANDWIDTH_MARGIN_MIN;            // %.

protected:
    const int TRIGGER_SOFTWARE;
    const int EXPOSURE_MILLISEC_BASE;

    const int CAM_IDX_0;
    const int CAM_IDX_1;

    const int XI_DEFAULT_TOTAL_BANDWIDTH;
    const int XI_DEFAULT_BANDWIDTH_MARGIN;

    std::string mCamSN[N_XI_C];

    xiAPIplusCameraOcv mCams[N_XI_C];

    double mXi_AutoGainExposurePriority;
    int    mXi_AutoExposureTopLimit;     // Milisecond.
    int    mXi_TotalBandwidth;           // MBit/s.
    int    mXi_BandwidthMargin;          // %.
    double mXi_MaxFrameRate;             // fps.
};

}

#endif /* __STEREOXICAMERA_H__ */