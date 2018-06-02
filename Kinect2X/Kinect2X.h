//    File: Kinect2X.h
//
//	  Date: April, 2015
//
//  Author: Marek Jakab

#ifndef KCV_SENSOR_H
#define KCV_SENSOR_H

// Kinect2X.h

// Kinect SDK
#include <Kinect.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/contrib/contrib.hpp>
#include <opencv2/imgproc/imgproc.hpp>


namespace kcv
{
	class KCV_sensor
	{
	public:
		static KCV_sensor *getInstance()
		{
			static KCV_sensor *instance = new KCV_sensor();
			return instance;
		}
		

		void closeAll();

		HRESULT initSensor();
		HRESULT initSensor(int c_width, int c_height, int d_width, int d_height);
		void setCoordinateMapper(const cv::Mat &colorImage,const cv::Mat &depthImage);
		bool isTraining = false;

		// Depth frame reader
		IDepthFrameReader *m_DepthFrameReader;
		// Color frame reader
		IColorFrameReader *m_ColorFrameReader;
		// Multi reader
		IMultiSourceFrameReader *m_MultiSourceFrameReader;

		void acquireColorImage(IColorFrame **color_frame);
		HRESULT acquireColorImage(cv::Mat &color_frame);
		void acquireDepthImage(IDepthFrame **depth_frame);
		void acquireRealDepthImage(cv::Mat &depth_frame);
		HRESULT acquireVisDepthImage(cv::Mat &depth_frame);
		HRESULT acquireImages(cv::Mat &depth_frame, cv::Mat &color_frame);
		void KCV_sensor::visualiseDepthMap(cv::Mat depth_frame, cv::Mat &depth_frame_vis);
		bool isAvailable();

		//KCV_sensor::
		void alignColorFrame(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
			const RGBQUAD* pColorBuffer, int nColorWidth, int nColorHeight, cv::Mat &aligned_color_frame);
		void alignColorFrame(int nDepthWidth, int nDepthHeight, cv::Mat color_frame,
			int nColorWidth, int nColorHeight, cv::Mat &aligned_color_frame, int aligned_frame_width, int aligned_frame_height);
		void KCV_sensor::alignIntensityFrame(int nDepthWidth, int nDepthHeight,	cv::Mat intensity_frame, int nIntensityWidth, 
			int nIntensityHeight, cv::Mat &aligned_intensity_frame, int aligned_frame_width, int aligned_frame_height);
		void alignDepthFrame(cv::Mat depth_frame, int nDepthWidth, int nDepthHeight,
			int nColorWidth, int nColorHeight, cv::Mat &aligned_depth_frame, int aligned_frame_width, int aligned_frame_height);
		void alignDepthFrame(const UINT16* pDepthBuffer, int nDepthWidth, int nDepthHeight,
			int nColorWidth, int nColorHeight, cv::Mat &aligned_depth_frame);
		bool getPointInDepth(cv::Point colorPoint, int nColorWidth, int nColorHeight,
			int nDepthWidth, int nDepthHeight, cv::Point &depthPoint);
		bool getPointInReal(cv::Point depthPoint, int nDepthWidth, int nDepthHeight, cv::Point3f &realPoint);
		bool getPointFromReal(cv::Point3f realPoint, int nDepthWidth, int nDepthHeight, cv::Point &depthPoint);
		HRESULT mapDepthFrameToCameraSpace(cv::Mat depthImage, int nDepthWidth, int nDepthHeight);

	private:
		// Private Constructor
		KCV_sensor();
		~KCV_sensor(); //virtual 

		KCV_sensor(const KCV_sensor&);//KCV_sensor const& copy);
		KCV_sensor& operator=(const KCV_sensor&);
		//KCV_sensor& operator=(KCV_sensor const& copy);

		HRESULT initialize();
		HRESULT openKinectDevice();
		HRESULT openDepthStream();
		HRESULT openColorStream();
		HRESULT openMultiStream();

		void release(int index);

		// Kinect sensor
		IKinectSensor *m_KinectSensor;
		DepthSpacePoint *m_DepthCoordinates;
		ColorSpacePoint *m_ColorCoordinates;
		CameraSpacePoint *m_CameraCoordinates;
		ICoordinateMapper *m_CoordinateMapper;

		// Images
		IColorFrame *c_frame;
		IFrameDescription *c_fd;
		float c_frame_width_scale;
		float c_frame_heigth_scale;

		IDepthFrame *d_frame;
		IFrameDescription *d_fd;
		float d_frame_width_scale;
		float d_frame_heigth_scale;

		bool coordMapped;

		HRESULT coordinateMapper(const UINT16* p_DepthBuffer, int nDepthWidth, int nDepthHeight,
			const RGBQUAD* p_ColorBuffer, int nColorWidth, int nColorHeight);

		HRESULT status;

	};
}

#endif // KCV_SENSOR_H