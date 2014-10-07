#ifndef KCV_SENSOR_H
#define KCV_SENSOR_H

// kcv_sensor.h

// Kinect SDK
#include <Kinect.h>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Internal headers
#include "kcv_depth.h"


namespace kcv
{
	class KCV_sensor
	{
	public:
		KCV_sensor();
		~KCV_sensor();
	private:

		HRESULT initialize();
		HRESULT openKinectDevice();
		HRESULT openDepthStream();
		HRESULT openColorStream();

		void release(int index);

		// Kinect sensor
		IKinectSensor *m_KinectSensor;
		// Depth frame reader
		IDepthFrameReader *m_DepthFrameReader;
		// Color frame reader
		IColorFrameReader *m_ColorFrameReader;

		// Depth image processing with OpenCV
		KCV_depth *m_KCVdepth;
	};
}

#endif // KCV_SENSOR_H