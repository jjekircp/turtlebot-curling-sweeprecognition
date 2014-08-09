//-----------------------------------------------------------------------------
// <copyright file="OpenCVFrameHelper.cpp" company="Microsoft">
//     Copyright (c) Microsoft Corporation. All rights reserved.
// </copyright>
//-----------------------------------------------------------------------------

#include "OpenCVFrameHelper.h"
#include <opencv2/video/tracking.hpp>
#include <iostream>

#pragma comment (lib, "opencv_core249d.lib")
#pragma comment (lib, "opencv_highgui249d.lib")
#pragma comment (lib, "opencv_imgproc249d.lib")
#pragma comment (lib, "opencv_video249d.lib")
#pragma comment (lib, "opencv_features2d249d.lib")


using namespace Microsoft::KinectBridge;

/// <summary>
/// Converts from Kinect color frame data into a RGB OpenCV image matrix. 
/// User must pre-allocate space for matrix.
/// </summary>
/// <param name="pImage">pointer in which to return the OpenCV image matrix</param>
/// <returns>S_OK if successful, an error code otherwise</returns>
HRESULT OpenCVFrameHelper::GetColorData(Mat* pImage) const
{
    // Check if image is valid
    if (m_colorBufferPitch == 0)
    {
        return E_NUI_FRAME_NO_DATA;
    }

    DWORD colorHeight, colorWidth;
    NuiImageResolutionToSize(m_colorResolution, colorWidth, colorHeight);

    // Copy image information into Mat
    for (UINT y = 0; y < colorHeight; ++y)
    {
        // Get row pointer for color Mat
        Vec4b* pColorRow = pImage->ptr<Vec4b>(y);

        for (UINT x = 0; x < colorWidth; ++x)
        {
            pColorRow[x] = Vec4b(m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 0],
                m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 1],
                m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 2],
                m_pColorBuffer[y * m_colorBufferPitch + x * 4 + 3]);
        }
    }

    return S_OK;
}

/// <summary>
/// Converts from Kinect depth frame data into a OpenCV matrix
/// User must pre-allocate space for matrix.
/// </summary>
/// <param name="pImage">pointer in which to return the OpenCV matrix</param>
/// <returns>S_OK if successful, an error code otherwise</returns>
HRESULT OpenCVFrameHelper::GetDepthData(Mat* pImage) const
{
    // Check if image is valid
    if (m_colorBufferPitch == 0)
    {
        return E_NUI_FRAME_NO_DATA;
    }

    DWORD depthHeight, depthWidth;
    NuiImageResolutionToSize(m_depthResolution, depthWidth, depthHeight);

    // Copy image information into Mat
    USHORT* pBufferRun = reinterpret_cast<USHORT*>(m_pDepthBuffer);

    for (UINT y = 0; y < depthHeight; ++y)
    {
        // Get row pointer for depth Mat
        USHORT* pDepthRow = pImage->ptr<USHORT>(y);

        for (UINT x = 0; x < depthWidth; ++x)
        {
            pDepthRow[x] = pBufferRun[y * depthWidth + x];
        }
    }

    return S_OK;
}

/// <summary>
/// Converts from Kinect depth frame data into a ARGB OpenCV image matrix
/// User must pre-allocate space for matrix.
/// </summary>
/// <param name="pImage">pointer in which to return the OpenCV image matrix</param>
/// <returns>S_OK if successful, an error code otherwise</returns>
HRESULT OpenCVFrameHelper::GetDepthDataAsArgb(Mat* pImage) const
{
	static Mat prevImage;
    DWORD depthWidth, depthHeight;
	static std::vector<cv::Point> prevFeatures;
	static std::vector<cv::Point> currentFeatures(20);

    NuiImageResolutionToSize(m_depthResolution, depthWidth, depthHeight);

    // Get the depth image
    Mat depthImage;
    depthImage.create(depthHeight, depthWidth, DEPTH_TYPE);
    HRESULT hr = GetDepthData(&depthImage);
    if (!SUCCEEDED(hr)) {
        return hr;
    }
	Mat testImage(depthImage);
	testImage.convertTo(testImage, CV_8U);
	if (prevFeatures.size() <= 0) {
		try {
			cv::goodFeaturesToTrack(testImage, currentFeatures, 20, 0.05, 5);
		}
		catch (cv::Exception ex) {
			std::cerr << "Exception: " << ex.what() << std::endl;
		}
	}
	else {
		vector<uchar> status;
		vector<float> err;
		try {
			cv::calcOpticalFlowPyrLK(prevImage, testImage, prevFeatures, currentFeatures,
				status, err);
		}
		catch (cv::Exception ex) {
			std::cout << "Exception: " << ex.what() << std::endl;
		}
	}
    for (UINT y = 0; y < depthHeight; ++y)
    {
        // Get row pointers for Mats
        const USHORT* pDepthRow = depthImage.ptr<USHORT>(y);
        Vec4b* pDepthRgbRow = pImage->ptr<Vec4b>(y);

        for (UINT x = 0; x < depthWidth; ++x)
        {
            USHORT raw_depth = pDepthRow[x];

            // If depth value is valid, convert and copy it
            if (raw_depth != 65535)
            {
                UINT8 redPixel, greenPixel, bluePixel;
                DepthShortToRgb(raw_depth, &redPixel, &greenPixel, &bluePixel);
                pDepthRgbRow[x] = Vec4b(redPixel, greenPixel, bluePixel, 1);
            }
            else
            {
                pDepthRgbRow[x] = 0;
            }
        }
    }

	prevImage = testImage.clone();
	prevFeatures = currentFeatures;
    return S_OK;
}

/// <summary>
/// Verify image is of the given resolution
/// </summary>
/// <param name="pImage">pointer to image to verify</param>
/// <param name="resolution">resolution of image</param>
/// <returns>S_OK if image matches given width and height, an error code otherwise</returns>
HRESULT OpenCVFrameHelper::VerifySize(const Mat* pImage, NUI_IMAGE_RESOLUTION resolution) const
{
    DWORD width, height;
    NuiImageResolutionToSize(resolution, width, height);

    Size size = pImage->size();
    if (size.height != height || size.width != width)
    {
        return E_INVALIDARG;
    }

    return S_OK;
}

