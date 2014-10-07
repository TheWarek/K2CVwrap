
#include "kcv_sensor.h"

using namespace kcv;

KCV_sensor::KCV_sensor()
{
	HRESULT hr = this->initialize();
	hr = openKinectDevice();
	hr = openDepthStream();
	hr = openColorStream();
}

HRESULT KCV_sensor::initialize()
{
	HRESULT hr;
	hr = GetDefaultKinectSensor(&this->m_KinectSensor);
	if (FAILED(hr))
	{
		return hr;

	}
	return hr;
}

HRESULT KCV_sensor::openKinectDevice()
{
	HRESULT hr;
	hr = this->m_KinectSensor->Open(); // OpenMultiSourceFrameReader - need to specify type

	if (!this->m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	return hr;
}

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

		m_ColorFrameSource->Release();
	}

	if (!this->m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	return hr;
}

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

		m_DepthFrameSource->Release();
	}

	if (!this->m_KinectSensor || FAILED(hr))
	{
		return E_FAIL;
	}
	return hr;
}