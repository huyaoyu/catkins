#include <iostream>

#include "StereoXiCamera.hpp"

using namespace sxc;

StereoXiCamera::StereoXiCamera(std::string &camSN0, std::string &camSN1)
: TRIGGER_SOFTWARE(1), EXPOSURE_MILLISEC_BASE(1000), CAM_IDX_0(0), CAM_IDX_1(1),
  XI_DEFAULT_TOTAL_BANDWIDTH(2400), XI_DEFAULT_BANDWIDTH_MARGIN(10)
{
    mCamSN[CAM_IDX_0] = camSN0;
    mCamSN[CAM_IDX_1] = camSN1;
}

StereoXiCamera::~StereoXiCamera()
{

}

void StereoXiCamera::open()
{
    prepare_before_opening();
    open_and_common_settings();
}

void StereoXiCamera::start_acquisition(int waitMS)
{
    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].StartAcquisition();
    LOOP_CAMERAS_END

    // Wait for a short period of time.
    cvWaitKey(waitMS);
}

void StereoXiCamera::software_trigger(void)
{
    // Trigger.
    mCams[CAM_IDX_0].SetTriggerSoftware(TRIGGER_SOFTWARE);
}

void StereoXiCamera::get_images(cv::Mat &img0, cv::Mat &img1)
{
    img0 = get_single_image(CAM_IDX_0);
    img1 = get_single_image(CAM_IDX_1);
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

void StereoXiCamera::stop_acquisition(int waitMS)
{
    // Stop acquisition.
    LOOP_CAMERAS_BEGIN
        mCams[loopIdx].StopAcquisition();
    LOOP_CAMERAS_END

    cvWaitKey(waitMS);
}

void StereoXiCamera::close()
{
    LOOP_CAMERAS_REVERSE_BEGIN
        mCams[loopIdx].Close();
    LOOP_CAMERAS_REVERSE_END
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
	cam.SetAutoExposureTopLimit( EXPOSURE_MILLISEC( mXi_AutoExposureTopLimit ) );
	cam.EnableAutoExposureAutoGain();

	// Enable auto-whitebalance.
	cam.EnableWhiteBalanceAuto();

	// Image format.
	cam.SetImageDataFormat(XI_RGB24);

	// Sensor defects selector.
	cam.SetSensorDefectsCorrectionListSelector(XI_SENS_DEFFECTS_CORR_LIST_SEL_USER0);
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

// =============++++== Getters and setters. =========================

void StereoXiCamera::set_autogain_exposure_priority(double val)
{
    mXi_AutoGainExposurePriority = val;
}

double StereoXiCamera::get_autogain_exposure_priority(void)
{
    return mXi_AutoGainExposurePriority;
}

void StereoXiCamera::set_autoexposure_top_limit(int tLimit)
{
    mXi_AutoExposureTopLimit = tLimit;
}

int StereoXiCamera::get_autoexposure_top_limit(void)
{
    return mXi_AutoExposureTopLimit;
}

void StereoXiCamera::set_total_bandwidth(int b)
{
    mXi_TotalBandwidth = b;
}

int StereoXiCamera::get_total_bandwidth(void)
{
    return mXi_TotalBandwidth;
}

void StereoXiCamera::set_bandwidth_margin(int m)
{
    mXi_BandwidthMargin = m;
}

int StereoXiCamera::get_bandwidth_margin(void)
{
    return mXi_BandwidthMargin;
}

double StereoXiCamera::get_max_frame_rate(void)
{
    return mXi_MaxFrameRate;
}