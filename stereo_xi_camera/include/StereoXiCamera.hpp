#ifndef __STEREOXICAMERA_H__
#define __STEREOXICAMERA_H__

#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "xiApiPlusOcv.hpp"

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

namespace sxc {

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
    int    mXi_TotalBandwidth;           // Mbits/s.
    int    mXi_BandwidthMargin;          // %.
    double mXi_MaxFrameRate;             // fps.
};

}

#endif /* __STEREOXICAMERA_H__ */