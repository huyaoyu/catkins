#include <iostream>
#include <sstream>

#include "StereoXiCamera.hpp"

using namespace sxc;

StereoXiCamera::StereoXiCamera(std::string &camSN0, std::string &camSN1)
: AUTO_GAIN_EXPOSURE_PRIORITY_MAX(1.0), AUTO_GAIN_EXPOSURE_PRIORITY_MIM(0.49), AUTO_GAIN_EXPOSURE_PRIORITY_DEFAULT(0.8),
  AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX(60.0), AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN(10.0), AUTO_GAIN_EXPOSURE_TARGET_LEVEL_DEFAULT(40.0),
  AUTO_EXPOSURE_TOP_LIMIT_MAX(1000), AUTO_EXPOSURE_TOP_LIMIT_MIN(1), AUTO_EXPOSURE_TOP_LIMIT_DEFAULT(200),
  AUTO_GAIN_TOP_LIMIT_MAX(36.0), AUTO_GAIN_TOP_LIMIT_MIN(0.0), AUTO_GAIN_TOP_LIMIT_DEFAULT(12.0),
  TOTAL_BANDWIDTH_MAX(4000), TOTAL_BANDWIDTH_MIN(2400),
  BANDWIDTH_MARGIN_MAX(50), BANDWIDTH_MARGIN_MIN(5), BANDWIDTH_MARGIN_DEFAULT(10),
  TRIGGER_SOFTWARE(1), EXPOSURE_MILLISEC_BASE(1000), CAM_IDX_0(0), CAM_IDX_1(1),
  XI_DEFAULT_TOTAL_BANDWIDTH(2400), XI_DEFAULT_BANDWIDTH_MARGIN(10),
  mXi_AutoGainExposurePriority(AUTO_GAIN_EXPOSURE_PRIORITY_DEFAULT),
  mXi_AutoExposureTopLimit(AUTO_EXPOSURE_TOP_LIMIT_DEFAULT),
  mXi_AutoGainTopLimit(AUTO_GAIN_TOP_LIMIT_DEFAULT),
  mXi_BandwidthMargin(BANDWIDTH_MARGIN_DEFAULT)
{
    mCamSN[CAM_IDX_0] = camSN0;
    mCamSN[CAM_IDX_1] = camSN1;
}

StereoXiCamera::~StereoXiCamera()
{

}

void StereoXiCamera::open()
{
    try
    {
        prepare_before_opening();
        open_and_common_settings();
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

void StereoXiCamera::start_acquisition(int waitMS)
{
    try
    {
        LOOP_CAMERAS_BEGIN
            mCams[loopIdx].StartAcquisition();
        LOOP_CAMERAS_END
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }

    // Wait for a short period of time.
    cvWaitKey(waitMS);
}

void StereoXiCamera::software_trigger(void)
{
    try
    {
        // Trigger.
        mCams[CAM_IDX_0].SetTriggerSoftware(TRIGGER_SOFTWARE);
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

void StereoXiCamera::get_images(cv::Mat &img0, cv::Mat &img1)
{
    try
    {
        img0 = get_single_image(CAM_IDX_0);
        img1 = get_single_image(CAM_IDX_1);
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

void StereoXiCamera::get_images(cv::Mat &img0, cv::Mat &img1, CameraParams_t &camP0, CameraParams_t &camP1)
{
    this->get_images(img0, img1);

    try
    {
        put_single_camera_params( mCams[CAM_IDX_0], camP0 );
        put_single_camera_params( mCams[CAM_IDX_1], camP1 );
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

cv::Mat StereoXiCamera::get_single_image(int idx)
{
    // Obtain the images.
    XI_IMG_FORMAT format;
    cv::Mat cv_mat_image;

    format = mCams[idx].GetImageDataFormat();
    cv_mat_image = mCams[idx].GetNextImageOcvMat();
    std::cout << "format = " << format << std::endl;
    if (format == XI_RAW16 || format == XI_MONO16)
    {
        normalize(cv_mat_image, cv_mat_image, 0, 65536, cv::NORM_MINMAX, -1, cv::Mat()); // 0 - 65536, 16 bit unsigned integer range
    }

    return cv_mat_image;
}

void StereoXiCamera::put_single_camera_params(xiAPIplusCameraOcv &cam, CameraParams_t &cp)
{
    if ( true == cam.IsAutoExposureAutoGain() )
    {
        cp.AEAGEnabled = 1;
    }
    else
    {
        cp.AEAGEnabled = 0;
    }

    cp.AEAGPriority = (xf)( cam.GetAutoExposureAutoGainExposurePriority());

    cp.exposure = (int)( cam.GetExposureTime() / 1000.0 );

    cp.gain = (xf)( cam.GetGain() );
}

void StereoXiCamera::stop_acquisition(int waitMS)
{
    try
    {
        // Stop acquisition.
        LOOP_CAMERAS_BEGIN
            mCams[loopIdx].StopAcquisition();
        LOOP_CAMERAS_END
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }

    cvWaitKey(waitMS);
}

void StereoXiCamera::close()
{
    try
    {
        LOOP_CAMERAS_REVERSE_BEGIN
            mCams[loopIdx].Close();
        LOOP_CAMERAS_REVERSE_END
    }
    catch ( xiAPIplus_Exception& exp )
    {
        EXCEPTION_CAMERA_API(exp);
    }
}

void StereoXiCamera::put_sensor_filter_array(int idx, std::string &strFilterArray)
{
    XI_COLOR_FILTER_ARRAY filterArray = mCams[idx].GetSensorColorFilterArray();

    switch ( filterArray )
    {
        case (XI_CFA_NONE):
        {
            strFilterArray = "none";
            break;
        }
        case (XI_CFA_BAYER_RGGB):
        {
            strFilterArray = "bayer_rggb8";
            break;
        }
        case (XI_CFA_CMYG):
        {
            strFilterArray = "cmyg";
            break;
        }
        case (XI_CFA_RGR):
        {
            strFilterArray = "rgr";
            break;
        }
        case (XI_CFA_BAYER_BGGR):
        {
            strFilterArray = "bayer_bggr8";
            break;
        }
        case (XI_CFA_BAYER_GRBG):
        {
            strFilterArray = "bayer_grbg8";
            break;
        }
        case (XI_CFA_BAYER_GBRG):
        {
            strFilterArray = "bayer_gbrg8";
            break;
        }
        default:
        {
            // Should never reach here.
            strFilterArray = "error";
            break;
        }
    }
}

void StereoXiCamera::prepare_before_opening()
{
    // Bandwidth.
    LOOP_CAMERAS_BEGIN
	    mCams[loopIdx].DisableAutoBandwidthCalculation();
    LOOP_CAMERAS_END
}

void StereoXiCamera::open_and_common_settings()
{
    // Configure common parameters.
    LOOP_CAMERAS_BEGIN
        char *cstr = new char[mCamSN[loopIdx].length() + 1];
        strcpy(cstr, mCamSN[loopIdx].c_str());

        mCams[loopIdx].OpenBySN(cstr);
        setup_camera_common(mCams[loopIdx]);

        delete [] cstr; cstr = SXC_NULL;
    LOOP_CAMERAS_END

    // Configure synchronization.
    // Set trigger mode on the first camera - as master.
    mCams[CAM_IDX_0].SetTriggerSource(XI_TRG_SOFTWARE);
    mCams[CAM_IDX_0].SetGPOSelector(XI_GPO_PORT1);
    mCams[CAM_IDX_0].SetGPOMode(XI_GPO_EXPOSURE_ACTIVE);

    // Set trigger mode on the second camera - as slave.
    mCams[CAM_IDX_1].SetGPISelector(XI_GPI_PORT1);
    mCams[CAM_IDX_1].SetGPIMode(XI_GPI_TRIGGER);
    mCams[CAM_IDX_1].SetTriggerSource(XI_TRG_EDGE_RISING);
}

void StereoXiCamera::setup_camera_common(xiAPIplusCameraOcv& cam)
{
    // Set exposure time.
	cam.SetAutoExposureAutoGainExposurePriority( mXi_AutoGainExposurePriority );
    cam.SetAutoExposureAutoGainTargetLevel(mXi_AutoGainExposureTargetLevel);
	cam.SetAutoExposureTopLimit( EXPOSURE_MILLISEC( mXi_AutoExposureTopLimit ) );
    cam.SetAutoGainTopLimit( mXi_AutoGainTopLimit );
    cam.EnableAutoExposureAutoGain();

	// Enable auto-whitebalance.
	cam.EnableWhiteBalanceAuto();

	// Image format.
	cam.SetImageDataFormat(XI_RGB24);
	// cam.SetImageDataFormat(XI_RAW8);

	// Sensor defects selector.
	cam.SetSensorDefectsCorrectionListSelector(XI_SENS_DEFFECTS_CORR_LIST_SEL_USER0);
	cam.EnableSensorDefectsCorrection();
    cam.SetSensorDefectsCorrectionListSelector(XI_SENS_DEFFECTS_CORR_LIST_SEL_FACTORY);
	cam.EnableSensorDefectsCorrection();

	// Bandwith.
	int cameraDataRate = (int)( mXi_TotalBandwidth / 2.0 * ( 100.0 - mXi_BandwidthMargin ) / 100 );
    mXi_MaxFrameRate = mXi_TotalBandwidth / 2.0 / cameraDataRate;
	cam.SetBandwidthLimit( cameraDataRate );
}

int StereoXiCamera::EXPOSURE_MILLISEC(int val)
{
    return val * EXPOSURE_MILLISEC_BASE;
}

// ================== Getters and setters. =========================

void StereoXiCamera::set_autogain_exposure_priority(xf val)
{
    if ( val < AUTO_GAIN_EXPOSURE_PRIORITY_MIM ||
         val > AUTO_GAIN_EXPOSURE_PRIORITY_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(val, AUTO_GAIN_EXPOSURE_PRIORITY_MIM, AUTO_GAIN_EXPOSURE_PRIORITY_MAX);
    }
    else
    {
        mXi_AutoGainExposurePriority = val;
    }
}

xf StereoXiCamera::get_autogain_exposure_priority(void)
{
    return mXi_AutoGainExposurePriority;
}

void StereoXiCamera::set_autogain_exposure_target_level(xf val)
{
    if ( val < AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN ||
         val > AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(val, AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MIN, AUTO_GAIN_EXPOSURE_TARGET_LEVEL_MAX);
    }
    else
    {
        mXi_AutoGainExposureTargetLevel = val;
    }
}

xf StereoXiCamera::get_autogain_exposure_target_level(void)
{
    return mXi_AutoGainExposureTargetLevel;
}

void StereoXiCamera::set_autoexposure_top_limit(int tLimit)
{
    if ( tLimit < AUTO_EXPOSURE_TOP_LIMIT_MIN ||
         tLimit > AUTO_EXPOSURE_TOP_LIMIT_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(tLimit, AUTO_EXPOSURE_TOP_LIMIT_MIN, AUTO_EXPOSURE_TOP_LIMIT_MAX);
    }
    else
    {
        mXi_AutoExposureTopLimit = tLimit;
    }
}

int StereoXiCamera::get_autoexposure_top_limit(void)
{
    return mXi_AutoExposureTopLimit;
}

void StereoXiCamera::set_autogain_top_limit(xf tG)
{
    if ( tG < AUTO_GAIN_TOP_LIMIT_MIN ||
         tG > AUTO_GAIN_TOP_LIMIT_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(tG, AUTO_GAIN_TOP_LIMIT_MIN, AUTO_GAIN_TOP_LIMIT_MAX);
    }
    else
    {
        mXi_AutoGainTopLimit = tG;
    }
}

xf StereoXiCamera::get_autogain_top_limit(void)
{
    return mXi_AutoGainTopLimit;
}

void StereoXiCamera::set_total_bandwidth(int b)
{
    if ( b < TOTAL_BANDWIDTH_MIN ||
         b > TOTAL_BANDWIDTH_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(b, TOTAL_BANDWIDTH_MIN, TOTAL_BANDWIDTH_MAX);
    }
    else
    {
        mXi_TotalBandwidth = b;
    }
}

int StereoXiCamera::get_total_bandwidth(void)
{
    return mXi_TotalBandwidth;
}

void StereoXiCamera::set_bandwidth_margin(int m)
{
    if ( m < BANDWIDTH_MARGIN_MIN ||
         m > BANDWIDTH_MARGIN_MAX )
    {
        EXCEPTION_ARG_OUT_OF_RANGE(m, BANDWIDTH_MARGIN_MIN, BANDWIDTH_MARGIN_MAX);
    }
    else
    {
        mXi_BandwidthMargin = m;
    }
}

int StereoXiCamera::get_bandwidth_margin(void)
{
    return mXi_BandwidthMargin;
}

xf StereoXiCamera::get_max_frame_rate(void)
{
    return mXi_MaxFrameRate;
}