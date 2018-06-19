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

    std::string mCamSN[N_XI_C];

    xiAPIplusCameraOcv mCams[N_XI_C];
};

}

#endif /* __STEREOXICAMERA_H__ */