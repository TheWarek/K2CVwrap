//    File: Kinect2X.h
//
//	  Date: April, 2015
//
//  Author: Marek Jakab

#include "Kinect2X.h"

using namespace kcv;

/*!
\class KCV_sensor
\brief The KCV_sensor class manages Kinect sensor.
*/

// Safe release for interfaces
template<class Interface>
inline void SafeRelease(Interface *& pInterfaceToRelease)
{
	if (pInterfaceToRelease != NULL)
	{
		pInterfaceToRelease->Release();
		pInterfaceToRelease = NULL;
	}
}

/*!
Constructs a KCV_sensor object.
*/
KCV_sensor::KCV_sensor()
{
	this->status = initialize();
}

/*!
Desctructor.
*/
KCV_sensor::~KCV_sensor()
{
	if (m_MultiSourceFrameReader != NULL)
		SafeRelease(m_MultiSourceFrameReader);
	BOOLEAN b;
	if (m_KinectSensor != NULL)
	{
		m_KinectSensor->get_IsOpen(&b);
		if (b)
		{
			m_KinectSensor->Close();
		}
		SafeRelease(m_KinectSensor);
	}
}

/*!
Basic sensor initialisation.
*/
HRESULT KCV_sensor::initSensor()
{

	m_DepthCoordinates = NULL;
	m_ColorCoordinates = NULL;
	m_CameraCoordinates = NULL;

	//this->status = this->initialize();
	return this->status;
}

/*!
Init sensor with provided information about color \a c_width , \a c_height and depth \a d_width , \a d_height for scaling factor.
*/
HRESULT KCV_sensor::initSensor(int c_width, int c_height, int d_width, int d_height)
{
	m_DepthCoordinates = new DepthSpacePoint[1920 * 1080];
	m_ColorCoordinates = new ColorSpacePoint[512 * 424];
	m_CameraCoordinates = new CameraSpacePoint[512 * 424];
	coordMapped = false;

	this->c_frame_width_scale = (float)(1920.0f / (float)c_width);
	this->c_frame_heigth_scale = (float)(1080.0f / (float)c_height);
	this->d_frame_width_scale = (float)(512.0f / (float)d_width);
	this->d_frame_heigth_scale = (float)(424.0f / (float)d_height);

	return S_OK;
}

/*!
Returns if the sensor is available to use.
*/
bool KCV_sensor::isAvailable()
{
	if (SUCCEEDED(this->status))
	{
		return true;
	}
	return false;
}

/*!
Map coordinates from color to depth frame using \a colorImage and \a depthImage.
*/
void KCV_sensor::setCoordinateMapper(const cv::Mat &colorImage,const cv::Mat &depthImage)
{
	if (!isTraining) return;

	coordinateMapper(reinterpret_cast<UINT16*>(depthImage.data), depthImage.cols, depthImage.rows,
		reinterpret_cast<RGBQUAD*>(colorImage.data), colorImage.cols, colorImage.rows);
}

/*!
Opens multistream to capture frames from.
*/
HRESULT KCV_sensor::openMultiStream()
{
	HRESULT hr;
	if (m_KinectSensor)
	{

		hr = m_KinectSensor->get_CoordinateMapper(&m_CoordinateMapper);

		if (SUCCEEDED(hr))
		{
			hr = m_KinectSensor->OpenMultiSourceFrameReader(
				FrameSourceTypes::FrameSourceTypes_Depth | FrameSourceTypes::FrameSourceTypes_Color, &m_MultiSourceFrameReader);
		}
	}

	if (!m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}

	return hr;
}

/*!
Initializes the sensor, get default kinect sensor.
*/
HRESULT KCV_sensor::initialize()
{
	HRESULT hr;
	hr = GetDefaultKinectSensor(&this->m_KinectSensor);
	if (FAILED(hr))
	{
		this->m_KinectSensor = NULL;
		return hr;
	}
	if (SUCCEEDED(hr))
	{
		hr = openKinectDevice();
	}
	if (SUCCEEDED(hr))
	{
		hr = openMultiStream();
	}
	return hr;
}

/*!
Try to open the device for capture.
*/
HRESULT KCV_sensor::openKinectDevice()
{
	HRESULT hr;
	hr = this->m_KinectSensor->Open();
	// OpenMultiSourceFrameReader - need to specify type

	if (!this->m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	return hr;
}

/*!
Open color stream of kinect sensor.
*/
HRESULT KCV_sensor::openColorStream()
{
	HRESULT hr;

	if (this->m_KinectSensor)
	{
		IColorFrameSource *m_ColorFrameSource;

		hr = this->m_KinectSensor->get_ColorFrameSource(&m_ColorFrameSource);

		if (SUCCEEDED(hr))
		{
			hr = m_ColorFrameSource->OpenReader(&this->m_ColorFrameReader);
		}
		SafeRelease(m_ColorFrameSource);
	}

	if (!this->m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	return hr;
}

/*!
Open depth stream of kinect sensor.
*/
HRESULT KCV_sensor::openDepthStream()
{
	HRESULT hr;

	if (this->m_KinectSensor)
	{
		IDepthFrameSource *m_DepthFrameSource;

		hr = this->m_KinectSensor->get_DepthFrameSource(&m_DepthFrameSource);

		if (SUCCEEDED(hr))
		{
			hr = m_DepthFrameSource->OpenReader(&this->m_DepthFrameReader);
		}
		SafeRelease(m_DepthFrameSource);
	}

	if (!this->m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	return hr;
}

/*!
Acquire depth \a depth_frame and color \a color_frame images from the sensor.
*/
HRESULT KCV_sensor::acquireImages(cv::Mat &depth_frame, cv::Mat &color_frame)
{
	if (!this->m_MultiSourceFrameReader)
	{
		return E_FAIL;
	}

	IMultiSourceFrame *p_MultiSourceFrame = NULL;
	IDepthFrame *p_DepthFrame = NULL;
	IColorFrame *p_ColorFrame = NULL;

	HRESULT hr = this->m_MultiSourceFrameReader->AcquireLatestFrame(&p_MultiSourceFrame);

	if (SUCCEEDED(hr))
	{
		IDepthFrameReference* p_DepthFrameReference = NULL;

		hr = p_MultiSourceFrame->get_DepthFrameReference(&p_DepthFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = p_DepthFrameReference->AcquireFrame(&p_DepthFrame);
		}

		SafeRelease(p_DepthFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IColorFrameReference* p_ColorFrameReference = NULL;

		hr = p_MultiSourceFrame->get_ColorFrameReference(&p_ColorFrameReference);
		if (SUCCEEDED(hr))
		{
			hr = p_ColorFrameReference->AcquireFrame(&p_ColorFrame);
		}

		SafeRelease(p_ColorFrameReference);
	}

	if (SUCCEEDED(hr))
	{
		IFrameDescription *p_DepthFrameDescription = NULL;
		int nDepthWidth = 0;
		int nDepthHeight = 0;
		UINT nDepthBufferSize = 0;
		UINT16 *p_DepthBuffer = NULL;
		USHORT nDepthMinReliableDistance = 0;
		USHORT nDepthMaxReliableDistance = 0;

		IFrameDescription* p_ColorFrameDescription = NULL;
		int nColorWidth = 0;
		int nColorHeight = 0;
		ColorImageFormat imageFormat = ColorImageFormat_None;
		UINT nColorBufferSize = 0;
		RGBQUAD *p_ColorBuffer = NULL;

		// get depth frame data

		if (SUCCEEDED(hr))
		{
			hr = p_DepthFrame->get_FrameDescription(&p_DepthFrameDescription);
		}
		if (SUCCEEDED(hr))
		{
			hr = p_DepthFrame->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
			if (SUCCEEDED(hr))
			{
				hr = p_DepthFrame->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
			}
		}

		if (SUCCEEDED(hr))
		{
			hr = p_DepthFrameDescription->get_Width(&nDepthWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_DepthFrameDescription->get_Height(&nDepthHeight);
		}

		if (SUCCEEDED(hr))
		{
			p_DepthBuffer = new UINT16[nDepthWidth * nDepthHeight];
			p_DepthFrame->CopyFrameDataToArray(nDepthHeight * nDepthWidth, p_DepthBuffer);
		}
		// for depth map visualisation
		if (SUCCEEDED(hr))
		{
			cv::Mat depthMap = cv::Mat(nDepthHeight, nDepthWidth, CV_16U, p_DepthBuffer);
			depth_frame = depthMap.clone();
		}

		// get color frame data

		if (SUCCEEDED(hr))
		{
			hr = p_ColorFrame->get_FrameDescription(&p_ColorFrameDescription);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_ColorFrameDescription->get_Width(&nColorWidth);
		}

		if (SUCCEEDED(hr))
		{
			hr = p_ColorFrameDescription->get_Height(&nColorHeight);
		}

		if (SUCCEEDED(hr))
		{
			p_ColorBuffer = new RGBQUAD[nColorHeight * nColorWidth];
			hr = p_ColorFrame->CopyConvertedFrameDataToArray(nColorHeight * nColorWidth * sizeof(RGBQUAD),
				reinterpret_cast<BYTE*>(p_ColorBuffer), ColorImageFormat_Bgra);
		}
		if (SUCCEEDED(hr))
		{
			cv::Mat image(nColorHeight, nColorWidth, CV_8UC4,
				reinterpret_cast<void*>(p_ColorBuffer));
			color_frame = image.clone();
		}
		if (SUCCEEDED(hr))
		{
			hr = coordinateMapper(p_DepthBuffer, nDepthWidth, nDepthHeight,
				p_ColorBuffer, nColorWidth, nColorHeight);
		}

		// release buffers
		if (p_ColorBuffer != NULL)
		{
			delete[] p_ColorBuffer;
			p_ColorBuffer = NULL;
		}
		if (p_DepthBuffer != NULL)
		{
			delete[] p_DepthBuffer;
			p_DepthBuffer = NULL;
		}

		SafeRelease(p_DepthFrameDescription);
		SafeRelease(p_ColorFrameDescription);
	}

	SafeRelease(p_DepthFrame);
	SafeRelease(p_ColorFrame);
	SafeRelease(p_MultiSourceFrame);

	return hr;
}

/*!
Visualise \a depth_frame to 8 bit \a depth_frame_vis.
*/
void KCV_sensor::visualiseDepthMap(cv::Mat depth_frame, cv::Mat &depth_frame_vis)
{
	cv::Mat img0 = cv::Mat::zeros(depth_frame.rows, depth_frame.cols, CV_8UC1);

	double scale = 255.0 / (4500 -
		500); // 50 cm a 4.5 m
	depth_frame.convertTo(img0, CV_8UC1, scale);
	applyColorMap(img0, depth_frame_vis, cv::COLORMAP_JET);
}

/*!
Set coordinate mapper for given \a p_DepthBuffer , \a nDepthWidth , \a nDepthHeight , \a p_colorBuffer , \a nColorWidth and \a nColorHeight.
*/
HRESULT KCV_sensor::coordinateMapper(const UINT16* p_DepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* p_ColorBuffer, int nColorWidth, int nColorHeight)
{
	HRESULT hr = m_CoordinateMapper->MapDepthFrameToColorSpace(nDepthWidth * nDepthHeight, (UINT16*)p_DepthBuffer, nDepthWidth * nDepthHeight, m_ColorCoordinates);
	if (SUCCEEDED(hr))
		hr = m_CoordinateMapper->MapColorFrameToDepthSpace(nDepthWidth * nDepthHeight, (UINT16*)p_DepthBuffer, nColorWidth * nColorHeight, m_DepthCoordinates);
	if (SUCCEEDED(hr))
		hr = m_CoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth * nDepthHeight, (UINT16*)p_DepthBuffer, nDepthWidth * nDepthHeight, m_CameraCoordinates);
	return hr;
}

/*!
Maps \a depthImage with set \a nDepthWidth and \a nDepthHeight to camera space.
*/
HRESULT KCV_sensor::mapDepthFrameToCameraSpace(cv::Mat depthImage, int nDepthWidth, int nDepthHeight)
{
	UINT16 *p_DepthBuffer = (UINT16*)depthImage.data;
	HRESULT hr = m_CoordinateMapper->MapDepthFrameToCameraSpace(nDepthWidth * nDepthHeight, (UINT16*)p_DepthBuffer, nDepthWidth * nDepthHeight, m_CameraCoordinates);
	return hr;
}

/*!
Align intensity frame \a aligned_intensity_frame with specified \a aligned_frame_width and \a aligned_frame_height based on
\a intensity_frame with specified \a nIntensityWidth and \a nIntensityHeight with provided \a nDepthWidth and \a nDepthHeight .
*/
void KCV_sensor::alignIntensityFrame(int nDepthWidth, int nDepthHeight,
	cv::Mat intensity_frame, int nIntensityWidth, int nIntensityHeight, cv::Mat &aligned_intensity_frame, int aligned_frame_width, int aligned_frame_height)
{
	UCHAR *m_pOutputX = new UCHAR[nDepthWidth * nDepthHeight];
	UCHAR *buffer = intensity_frame.data;

	for (int intensityIndex = 0; intensityIndex < (nDepthWidth * nDepthHeight); ++intensityIndex)
	{

		UCHAR pSrc = NULL;

		ColorSpacePoint p = m_ColorCoordinates[intensityIndex];

		// Values that are negative infinity means it is an invalid color to depth mapping 
		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
		{
			int intensityX = static_cast<int>(p.X + 0.5f);
			int intensityY = static_cast<int>(p.Y + 0.5f);

			if ((intensityX >= 0 && intensityX < nIntensityWidth) && (intensityY >= 0 && intensityY < nIntensityHeight))
			{
				// set source for copy to the pixel
				pSrc = buffer[intensityY*nIntensityWidth + intensityX];
			}
		}

		// write output
		if (pSrc != NULL)
		{
			m_pOutputX[intensityIndex] = pSrc;
		}
		else
			m_pOutputX[intensityIndex] = 0;
	}
	// write image
	cv::Mat image(nDepthHeight, nDepthWidth, CV_8UC1, reinterpret_cast<void*>(m_pOutputX));
	cv::resize(image, aligned_intensity_frame, cv::Size(aligned_frame_width, aligned_frame_height));
	image.release();

	delete[] m_pOutputX;
	m_pOutputX = NULL;
}

/*!
Align color frame to \a aligned_color_frame with \a aligned_frame_width and \a aligned_frame_height 
based on \a color_frame with \a nColorWidth and \a nColorHeight from color stream 
and \a nDepthWidth and \a nDepthHeight from depth stream
*/
void KCV_sensor::alignColorFrame(int nDepthWidth, int nDepthHeight,
	cv::Mat color_frame, int nColorWidth, int nColorHeight, cv::Mat &aligned_color_frame, int aligned_frame_width, int aligned_frame_height)
{
	RGBQUAD *m_pOutputRGBX = new RGBQUAD[nDepthWidth * nDepthHeight];
	uchar *buffer = color_frame.data;

	for (int colorIndex = 0; colorIndex < (nDepthWidth * nDepthHeight); ++colorIndex)
	{
		const RGBQUAD* pSrc = NULL;

		ColorSpacePoint p = m_ColorCoordinates[colorIndex];

		// Values that are negative infinity means it is an invalid color to depth mapping
		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
		{
			int colorX = static_cast<int>(p.X + 0.5f);
			int colorY = static_cast<int>(p.Y + 0.5f);

			if ((colorX >= 0 && colorX < nColorWidth) && (colorY >= 0 && colorY < nColorHeight))
			{
				// set source for copy to the color pixel
				RGBQUAD *q;
				q->rgbBlue = buffer[colorY*nColorWidth + colorX];
				q->rgbGreen = buffer[colorY*nColorWidth + colorX + 1];
				q->rgbRed = buffer[colorY*nColorWidth + colorX + 2];
				q->rgbReserved = buffer[colorY*nColorWidth + colorX + 3];
				pSrc = q;
			}
		}

		// write output
		if (pSrc != NULL)
		{
			m_pOutputRGBX[colorIndex] = *pSrc;
		}
	}
	// write image
	cv::Mat image(nDepthHeight, nDepthWidth, CV_8UC4, reinterpret_cast<void*>(m_pOutputRGBX));
	cv::resize(image, aligned_color_frame, cv::Size(aligned_frame_width, aligned_frame_height));
	image.release();

	delete[] m_pOutputRGBX;
	m_pOutputRGBX = NULL;
}

/*!
Align color frame to \a aligned_color_frame with \a nColorWidth and \a nColorHeight
based on \a p_ColorBuffer from color stream
and \a p_DepthBuffer with \a nDepthWidth and \a nDepthHeight from depth stream.
*/
void KCV_sensor::alignColorFrame(const UINT16* p_DepthBuffer, int nDepthWidth, int nDepthHeight,
	const RGBQUAD* p_ColorBuffer, int nColorWidth, int nColorHeight, cv::Mat &aligned_color_frame)
{
	RGBQUAD *m_pOutputRGBX = new RGBQUAD[nDepthWidth * nDepthHeight];

	for (int colorIndex = 0; colorIndex < (nDepthWidth * nDepthHeight); ++colorIndex)
	{
		const RGBQUAD* pSrc = NULL;

		ColorSpacePoint p = m_ColorCoordinates[colorIndex];

		// Values that are negative infinity means it is an invalid color to depth mapping
		if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
		{
			int colorX = static_cast<int>(p.X + 0.5f);
			int colorY = static_cast<int>(p.Y + 0.5f);

			if ((colorX >= 0 && colorX < nColorWidth) && (colorY >= 0 && colorY < nColorHeight))
			{
				// set source for copy to the color pixel
				pSrc = p_ColorBuffer + colorY*nColorWidth + colorX;
			}
		}

		// write output
		if (pSrc != NULL)
		{
			m_pOutputRGBX[colorIndex] = *pSrc;
		}
	}
	// write image
	cv::Mat image(nDepthHeight, nDepthWidth, CV_8UC4, reinterpret_cast<void*>(m_pOutputRGBX));
	aligned_color_frame = image.clone();
	image.release();

	delete[] m_pOutputRGBX;
	m_pOutputRGBX = NULL;
}

/*!
Align depth frame to \a aligned_depth_frame with \a aligned_frame_width and \a aligned_frame_height
based on \a nColorWidth and \a nColorHeight from color stream
and \a depth_frame with \a nDepthWidth and \a nDepthHeight from depth stream.
*/
void KCV_sensor::alignDepthFrame(cv::Mat depth_frame, int nDepthWidth, int nDepthHeight,
	int nColorWidth, int nColorHeight, cv::Mat &aligned_depth_frame, int aligned_frame_width, int aligned_frame_height)
{
	UINT16 *m_pOutputDepth = new UINT16[nColorWidth * nColorHeight];
	UINT16 *buffer = (UINT16*)depth_frame.data;
	for (int depthIndex = 0; depthIndex < (nColorWidth * nColorHeight); ++depthIndex)
	{
		UINT16 pSrc = 0;

		DepthSpacePoint p = m_DepthCoordinates[depthIndex];

		int depthX = static_cast<int>(p.X + 0.5f);
		int depthY = static_cast<int>(p.Y + 0.5f);

		if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
		{
			pSrc = buffer[depthY*nDepthWidth + depthX];
		}

		// write output
		if (pSrc != 0)
		{
			m_pOutputDepth[depthIndex] = pSrc;
		}
		else
		{
			m_pOutputDepth[depthIndex] = USHRT_MAX;
		}

	}
	// write image
	cv::Mat image(nColorHeight, nColorWidth, CV_16U, reinterpret_cast<void*>(m_pOutputDepth));
	cv::resize(image, aligned_depth_frame, cv::Size(aligned_frame_width, aligned_frame_height));
	image.release();

	delete[] m_pOutputDepth;
	m_pOutputDepth = NULL;
}

/*!
Align depth frame to \a aligned_depth_frame with \a nColorWidth and \a nColorHeight
based on \a p_DepthBuffer with \a nDepthWidth and \a nDepthHeight from depth stream.
*/
void KCV_sensor::alignDepthFrame(const UINT16* p_DepthBuffer, int nDepthWidth, int nDepthHeight,
	int nColorWidth, int nColorHeight, cv::Mat &aligned_depth_frame)
{
	UINT16 *m_pOutputDepth = new UINT16[nColorWidth * nColorHeight];
	for (int depthIndex = 0; depthIndex < (nColorWidth * nColorHeight); ++depthIndex)
	{
		const UINT16* pSrc = NULL;

		DepthSpacePoint p = m_DepthCoordinates[depthIndex];

		int depthX = static_cast<int>(p.X + 0.5f);
		int depthY = static_cast<int>(p.Y + 0.5f);

		if ((depthX >= 0 && depthX < nDepthWidth) && (depthY >= 0 && depthY < nDepthHeight))
		{
			pSrc = p_DepthBuffer + depthY*nDepthWidth + depthX;
		}

		// write output
		if (pSrc != NULL)
		{
			m_pOutputDepth[depthIndex] = *pSrc;
		}
	}
	// write image
	cv::Mat image(nColorHeight, nColorWidth, CV_16U, reinterpret_cast<void*>(m_pOutputDepth));
	aligned_depth_frame = image.clone();
	image.release();

	delete[] m_pOutputDepth;
	m_pOutputDepth = NULL;
}

/*!
Store depth point information in \a depthPoint based on \a colorPoint , \a nColorWidth , \a nColorHeight , \a nDepthWidth and
 \a nDepthHeight .
*/
bool KCV_sensor::getPointInDepth(cv::Point colorPoint, int nColorWidth, int nColorHeight,
	int nDepthWidth, int nDepthHeight, cv::Point &depthPoint)
{
	DepthSpacePoint p = m_DepthCoordinates[(int)(((int)colorPoint.y) *nColorWidth*this->c_frame_width_scale) + (int)((int)colorPoint.x * this->c_frame_width_scale)];

	int xDepth = -1;
	int yDepth = -1;

	if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
	{
		xDepth = static_cast<int>(p.X + 0.5f);
		yDepth = static_cast<int>(p.Y + 0.5f);
	}
	if ((xDepth >= 0 && xDepth < nDepthWidth* this->c_frame_width_scale) && (yDepth >= 0 && yDepth < nDepthHeight* this->c_frame_heigth_scale)) // zmena
	{
		depthPoint.x = xDepth;
		depthPoint.y = yDepth;
		return true;
	}
	depthPoint.x = xDepth;
	depthPoint.y = yDepth;
	return false;
}

/*!
Store real point coordinates information in \a realPoint based on 
\a depthPoint , \a nDepthWidth and \a nDepthHeight .
*/
bool KCV_sensor::getPointInReal(cv::Point depthPoint, int nDepthWidth, int nDepthHeight, cv::Point3f &realPoint)
{
	if (!(depthPoint.x >= 0 && depthPoint.x < nDepthWidth* this->d_frame_width_scale) && !(depthPoint.y >= 0 && depthPoint.y < nDepthHeight* this->d_frame_heigth_scale))
		return false;
	float a = this->c_frame_width_scale;
	float b = this->c_frame_heigth_scale;
	CameraSpacePoint p = m_CameraCoordinates[(int)(((int)depthPoint.y) *nDepthWidth * this->d_frame_width_scale) + (int)((int)depthPoint.x* this->d_frame_width_scale)];

	float xDepth = 0.0f;
	float yDepth = 0.0f;
	float zDepth = 0.0f;

	if (p.X != -std::numeric_limits<float>::infinity() && p.Y != -std::numeric_limits<float>::infinity())
	{
		realPoint.x = p.X;
		realPoint.y = p.Y;
		realPoint.z = p.Z;
		return true;
	}
	realPoint.x = xDepth;
	realPoint.y = yDepth;
	realPoint.z = zDepth;
	return false;
}

/*!
Store real point coordinates information in \a realPoint based on
\a depthPoint , \a nDepthWidth and \a nDepthHeight .
*/
bool KCV_sensor::getPointFromReal(cv::Point3f realPoint, int nDepthWidth, int nDepthHeight, cv::Point &depthPoint)
{
	CameraSpacePoint p;
	p.X = realPoint.x;
	p.Y = realPoint.y;
	p.Z = realPoint.z;
	DepthSpacePoint d;
	m_CoordinateMapper->MapCameraPointToDepthSpace(p, &d);
	if ((d.X >= 0 && d.X < nDepthWidth * this->d_frame_width_scale) && (d.Y >= 0 && d.Y < nDepthHeight* this->d_frame_heigth_scale))
	{
		depthPoint.x = d.X;
		depthPoint.y = d.Y;
		return true;
	}
	depthPoint.x = -1;
	depthPoint.y = -1;
	return false;
}

/*!
Acquire actual visualisation of /depth_frame .
*/
HRESULT KCV_sensor::acquireVisDepthImage(cv::Mat &depth_frame)
{
	IDepthFrame *data = NULL;
	IFrameDescription *frameDesc = NULL;
	HRESULT hr = -1;
	UINT16 *depthBuffer = NULL;
	USHORT nDepthMinReliableDistance = 0;
	USHORT nDepthMaxReliableDistance = 0;
	int height = 424, width = 512;

	hr = m_DepthFrameReader->AcquireLatestFrame(&data);
	if (!SUCCEEDED(hr))
	{
		SafeRelease(data);
		return hr;
	}
	//data->CopyFrameDataToArray();
	hr = data->get_FrameDescription(&frameDesc);
	if (!SUCCEEDED(hr))
	{
		SafeRelease(data);
		SafeRelease(frameDesc);
		return hr;
	}
	hr = data->get_DepthMinReliableDistance(&nDepthMinReliableDistance);
	if (!SUCCEEDED(hr))
	{
		SafeRelease(data);
		SafeRelease(frameDesc);
		return hr;
	}
	if (SUCCEEDED(hr)) {
		hr = data->get_DepthMaxReliableDistance(&nDepthMaxReliableDistance);
		if (SUCCEEDED(frameDesc->get_Height(&height)) &&
			SUCCEEDED(frameDesc->get_Width(&width))) {
			depthBuffer = new UINT16[height * width];
			hr = data->CopyFrameDataToArray(height * width, depthBuffer);
			//hr = data->CopyFrameDataToArray(height * width, d_frame_data);
			if (SUCCEEDED(hr)) {
				cv::Mat depthMap = cv::Mat(height, width, CV_16U, depthBuffer);
				cv::Mat img0 = cv::Mat::zeros(height, width, CV_8UC1);

				double scale = 255.0 / (nDepthMaxReliableDistance -
					nDepthMinReliableDistance);
				depthMap.convertTo(img0, CV_8UC1, scale);
				applyColorMap(img0, depth_frame, cv::COLORMAP_JET);
			}
		}
		SafeRelease(frameDesc);
	}
	if (depthBuffer != NULL) {
		delete[] depthBuffer;
		depthBuffer = NULL;
	}
	if (!SUCCEEDED(hr))
	{
		SafeRelease(data);
		SafeRelease(frameDesc);
		return hr;
	}
	SafeRelease(data);
	return hr;
}

/*!
Acquire actual \a color_frame .
*/
HRESULT KCV_sensor::acquireColorImage(cv::Mat &color_frame)
{
	IColorFrame *data = NULL;
	IFrameDescription *frameDesc = NULL;
	HRESULT hr = -1;
	RGBQUAD *colorBuffer = NULL;

	hr = m_ColorFrameReader->AcquireLatestFrame(&data);
	if (!SUCCEEDED(hr))
	{
		SafeRelease(data);
		return hr;
	}
	if (SUCCEEDED(hr)) {
		hr = data->get_FrameDescription(&frameDesc);
		int height = 0, width = 0;
		if (SUCCEEDED(frameDesc->get_Height(&height)) &&
			SUCCEEDED(frameDesc->get_Width(&width))) {
			colorBuffer = new RGBQUAD[height * width];
			hr = data->CopyConvertedFrameDataToArray(height * width * sizeof(RGBQUAD),
				reinterpret_cast<BYTE*>(colorBuffer), ColorImageFormat_Bgra);
			if (SUCCEEDED(hr)) {
				cv::Mat image(height, width, CV_8UC4,
					reinterpret_cast<void*>(colorBuffer));
				cv::resize(image, color_frame, cv::Size(512, 424));
			}
		}
		SafeRelease(frameDesc);
	}
	if (colorBuffer != NULL) {
		delete[] colorBuffer;
		colorBuffer = NULL;
	}
	if (!SUCCEEDED(hr))
	{
		SafeRelease(frameDesc);
		SafeRelease(data);
		return hr;
	}
	SafeRelease(data);
	return hr;
}

/*!
Close connection to kinect device.
*/
void KCV_sensor::closeAll()
{
	
	if (m_DepthCoordinates != NULL)
		delete[] m_DepthCoordinates;
	if (m_ColorCoordinates != NULL)
		delete[] m_ColorCoordinates;
	if (m_CameraCoordinates != NULL)
		delete[] m_CameraCoordinates;
	m_DepthCoordinates = NULL;
	m_ColorCoordinates = NULL;
	m_CameraCoordinates = NULL;
}