#include "camera.hpp"

cv::Mat Camera::transfer(MV_FRAME_OUT &raw) 
{
    MV_CC_PIXEL_CONVERT_PARAM cvt_param;
    cv::Mat img(cv::Size(raw.stFrameInfo.nWidth, raw.stFrameInfo.nHeight), CV_8U, raw.pBufAddr);

    cvt_param.nWidth = raw.stFrameInfo.nWidth;
    cvt_param.nHeight = raw.stFrameInfo.nHeight;
    cvt_param.pSrcData = raw.pBufAddr;
    cvt_param.nSrcDataLen = raw.stFrameInfo.nFrameLen;
    cvt_param.enSrcPixelType = raw.stFrameInfo.enPixelType;

    cvt_param.pDstBuffer = img.data;
    cvt_param.nDstBufferSize = img.total() * img.elemSize();
    cvt_param.enDstPixelType = PixelType_Gvsp_BGR8_Packed;

    auto pixel_type = raw.stFrameInfo.enPixelType;
    const static std::unordered_map<MvGvspPixelType, cv::ColorConversionCodes> type_map = {
        {PixelType_Gvsp_BayerGR8, cv::COLOR_BayerGR2RGB},
        {PixelType_Gvsp_BayerRG8, cv::COLOR_BayerRG2RGB},
        {PixelType_Gvsp_BayerGB8, cv::COLOR_BayerGB2RGB},
        {PixelType_Gvsp_BayerBG8, cv::COLOR_BayerBG2RGB},
    };
    cv::cvtColor(img, img, type_map.at(pixel_type));

    return img;
}

Camera::Camera() 
{
    MV_CC_DEVICE_INFO_LIST device_list;
    int ret = MV_CC_EnumDevices(MV_USB_DEVICE, &device_list);
    if (ret != MV_OK) {
        return; // 原代码是 return -1，这里是构造函数，直接 return
    }
    if (device_list.nDeviceNum == 0) {
        return;
    }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[0]);
    if (ret != MV_OK) {
        return;
    }

    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK) {
        return;
    }

    MV_CC_SetEnumValue(handle_, "BalanceWhiteAuto", MV_BALANCEWHITE_AUTO_CONTINUOUS);
    MV_CC_SetEnumValue(handle_, "ExposureAuto", MV_EXPOSURE_AUTO_MODE_OFF);
    MV_CC_SetEnumValue(handle_, "GainAuto", MV_GAIN_MODE_OFF);
    MV_CC_SetFloatValue(handle_, "ExposureTime", 10000);
    MV_CC_SetFloatValue(handle_, "Gain", 20);
    MV_CC_SetFrameRate(handle_, 30);

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK) {
        return;
    }
}

Camera::~Camera() 
{
    MV_CC_StopGrabbing(handle_);
    MV_CC_CloseDevice(handle_);
    MV_CC_DestroyHandle(handle_);
}

cv::Mat Camera::read() 
{
    MV_FRAME_OUT raw;
    unsigned int nMsec = 10;
    MV_FRAME_OUT tmp_raw;
    while (MV_CC_GetImageBuffer(handle_, &tmp_raw, 0) == MV_OK) 
    {
    MV_CC_FreeImageBuffer(handle_, &tmp_raw);
    }

    int ret = MV_CC_GetImageBuffer(handle_, &raw, nMsec);
    if (ret != MV_OK) {
        return cv::Mat(); // 原代码是 return -1，这里强制要求返回 cv::Mat，只能返回空的
    }

    cv::Mat img = transfer(raw);

    MV_CC_FreeImageBuffer(handle_, &raw);

    return img;
}