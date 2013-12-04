/*
* 
* Copyright (c) 2013, Wieden+Kennedy, Stephen Schieberl
* All rights reserved.
* 
* Redistribution and use in source and binary forms, with or 
* without modification, are permitted provided that the following 
* conditions are met:
* 
* Redistributions of source code must retain the above copyright 
* notice, this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
* notice, this list of conditions and the following disclaimer in 
* the documentation and/or other materials provided with the 
* distribution.
* 
* Neither the name of the Ban the Rewind nor the names of its 
* contributors may be used to endorse or promote products 
* derived from this software without specific prior written 
* permission.
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS 
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE 
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, 
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; 
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, 
* STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
* ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
* ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
* 
*/

#include "Kinect2.h"
#include "cinder/app/App.h"

#include <comutil.h>

namespace Kinect2
{
using namespace ci;
using namespace ci::app;
using namespace std;

string getStatusMessage( KinectStatus status )
{
	switch ( status ) {
	case KinectStatus::KinectStatus_Connected:
		return "Connected";
	case KinectStatus::KinectStatus_DeviceNotGenuine:
		return "Device not genuine";
	case KinectStatus::KinectStatus_DeviceNotSupported:
		return "Device not supported";
	case KinectStatus::KinectStatus_Disconnected:
		return "Disconnected";
	case KinectStatus::KinectStatus_Error:
		return "Error";
	case KinectStatus::KinectStatus_Initializing:
		return "Initializing";
	case KinectStatus::KinectStatus_InsufficientBandwidth:
		return "Insufficient bandwidth";
	case KinectStatus::KinectStatus_InUseAsExclusive:
		return "In use as exclusive";
	case KinectStatus::KinectStatus_InUseAsShared:
		return "In use as shared";
	case KinectStatus::KinectStatus_NotPowered:
		return "Not powered";
	case KinectStatus::KinectStatus_NotReady:
		return "Not ready";
	default:
		return "Undefined";
	}
}

DeviceOptions::DeviceOptions()
: mDeviceIndex( 0 ), mDeviceId( "" ), mEnabledAudio( false ), mEnabledBody( false ), 
mEnabledBodyIndex( false ), mEnabledColor( true ), mEnabledDepth( true ), 
mEnabledInfrared( false ), mEnabledInfraredLongExposure( false )
{
}

DeviceOptions& DeviceOptions::enableAudio( bool enable )
{
	mEnabledAudio = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableBody( bool enable )
{
	mEnabledBody = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableBodyIndex( bool enable )
{
	mEnabledBodyIndex = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableColor( bool enable )
{
	mEnabledColor = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableDepth( bool enable )
{
	mEnabledDepth = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfrared( bool enable )
{
	mEnabledInfrared = enable;
	return *this;
}

DeviceOptions& DeviceOptions::enableInfraredLongExposure( bool enable )
{
	mEnabledInfraredLongExposure = enable;
	return *this;
}

DeviceOptions& DeviceOptions::setDeviceId( const string& id )
{
	mDeviceId = id;
	return *this;
}

DeviceOptions& DeviceOptions::setDeviceIndex( int32_t index )
{
	mDeviceIndex = index;
	return *this;
}

const string& DeviceOptions::getDeviceId() const
{
	return mDeviceId;
}

int32_t	 DeviceOptions::getDeviceIndex() const
{
	return mDeviceIndex;
}

bool DeviceOptions::isAudioEnabled() const
{
	return mEnabledAudio;
}

bool DeviceOptions::isBodyEnabled() const
{
	return mEnabledBody;
}

bool DeviceOptions::isBodyIndexEnabled() const
{
	return mEnabledBodyIndex;
}

bool DeviceOptions::isColorEnabled() const
{
	return mEnabledColor;
}

bool DeviceOptions::isDepthEnabled() const
{
	return mEnabledDepth;
}

bool DeviceOptions::isInfraredEnabled() const
{
	return mEnabledInfrared;
}

bool DeviceOptions::isInfraredLongExposureEnabled() const
{
	return mEnabledInfraredLongExposure;
}

//////////////////////////////////////////////////////////////////////////////////////////////

Frame::Frame()
: mDeviceId( "" ), mTimeStamp( 0L )
{
}

Frame::Frame( long long time, const string& deviceId, const Surface8u& color,
			  const Channel16u& depth, const Channel16u& infrared, 
			  const Channel16u& infraredLongExposure )
: mSurfaceColor( color ), mChannelDepth( depth ), mChannelInfrared( infrared ), 
mChannelInfraredLongExposure( infraredLongExposure ), mDeviceId( deviceId ), 
mTimeStamp( time )
{
}

const Surface8u& Frame::getColor() const
{
	return mSurfaceColor;
}

const Channel16u& Frame::getDepth() const
{
	return mChannelDepth;
}

const string& Frame::getDeviceId() const
{
	return mDeviceId;
}

const Channel16u& Frame::getInfrared() const
{
	return mChannelInfrared;
}

const Channel16u& Frame::getInfraredLongExposure() const
{
	return mChannelInfraredLongExposure;
}

int64_t Frame::getTimeStamp() const
{
	return mTimeStamp;
}

//////////////////////////////////////////////////////////////////////////////////////////////

DeviceRef Device::create()
{
	return DeviceRef( new Device() );
}

Device::Device()
: mFrameReader( 0 ), mSensor( 0 ), mCoordinateMapper( 0 ), mStatus( KinectStatus::KinectStatus_Undefined )
{
	App::get()->getSignalUpdate().connect( bind( &Device::update, this ) );
}

Device::~Device()
{
	stop();
}

void Device::start( const DeviceOptions& deviceOptions )
{
	long hr = S_OK;
	mDeviceOptions = deviceOptions;
	
	IKinectSensorCollection* sensorCollection = 0;
	hr = GetKinectSensorCollection( &sensorCollection );
	if ( FAILED( hr ) || sensorCollection == 0 ) {
		// TODO throw exception
		return;
	}

	//sensorCollection->SubscribeCollectionChanged( &Device::onSensorCollectionChanged );
	IEnumKinectSensor* sensorEnum = 0;
	hr = sensorCollection->get_Enumerator( &sensorEnum );
	if ( FAILED( hr ) || sensorEnum == 0 ) {
		// TODO throw exception
		return;
	}

	hr			= S_OK;
	int32_t i	= 0;
	while ( SUCCEEDED( hr ) && i < 8 ) { // TODO find actual max device count
		hr = sensorEnum->GetNext( &mSensor );
		if ( mSensor != 0 ) {
			string id = "";
			wchar_t wid[ 48 ];
			if ( SUCCEEDED( mSensor->get_UniqueKinectId( 48, wid ) ) ) {
				id = wcharToString( wid );
			}

			if ( mDeviceOptions.getDeviceId().empty() ) {
				if ( mDeviceOptions.getDeviceIndex() == i ) {
					mDeviceOptions.setDeviceId( id );
					break;
				}
			} else {
				if ( mDeviceOptions.getDeviceId() == id ) {
					mDeviceOptions.setDeviceIndex( i );
					break;
				}
			}
		}
		++i;
	}

	if ( mSensor != 0 ) {
		//hr = mSensor->get_CoordinateMapper( &mCoordinateMapper );
		hr = mSensor->Open();
		if ( SUCCEEDED( hr ) ) {
			long flags = 0L;
			if ( mDeviceOptions.isAudioEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_Audio;
			}
			if ( mDeviceOptions.isBodyEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_Body;
			}
			if ( mDeviceOptions.isBodyIndexEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_BodyIndex;
			}
			if ( mDeviceOptions.isColorEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_Color;
			}
			if ( mDeviceOptions.isDepthEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_Depth;
			}
			if ( mDeviceOptions.isInfraredEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_Infrared;
			}
			if ( mDeviceOptions.isInfraredLongExposureEnabled() ) {
				flags |= FrameSourceTypes::FrameSourceTypes_LongExposureInfrared;
			}

			if ( flags & FrameSourceTypes_Color ) {
				console() << "Color enabled" << endl;
			}

			hr = mSensor->OpenMultiSourceFrameReader( flags, &mFrameReader );
			if ( FAILED( hr ) ) {
				if ( mFrameReader != 0 ) {
					mFrameReader->Release();
					mFrameReader = 0;
				}
			}
		}
	}
}

void Device::stop()
{
	if ( mCoordinateMapper != 0 ) {
		mCoordinateMapper->Release();
		mCoordinateMapper = 0;
	}
	if ( mFrameReader != 0 ) {
		mFrameReader->Release();
		mFrameReader = 0;
	}
	if ( mSensor != 0 ) {
		long hr = mSensor->Close();
		if ( SUCCEEDED( hr ) && mSensor != 0 ) {
			mSensor->Release();
			mSensor = 0;
		}
	}
}

const DeviceOptions& Device::getDeviceOptions() const
{
	return mDeviceOptions;
}

const Frame& Device::getFrame() const
{
	return mFrame;
}

KinectStatus Device::getStatus() const
{
	return mStatus;
}

void Device::update()
{
	if ( mSensor != 0 ) {
		mSensor->get_Status( &mStatus );
	}

	if ( mFrameReader == 0 ) {
		return;
	}

	IAudioBeamFrame* audioFrame								= 0;
	IBodyFrame* bodyFrame									= 0;
	IBodyIndexFrame* bodyIndexFrame							= 0;
	IColorFrame* colorFrame									= 0;
	IDepthFrame* depthFrame									= 0;
	IMultiSourceFrame* frame								= 0;
	IInfraredFrame* infraredFrame							= 0;
	ILongExposureInfraredFrame* infraredLongExposureFrame	= 0;
	
	HRESULT hr = mFrameReader->AcquireLatestFrame( &frame );

	// TODO audio
	if ( SUCCEEDED( hr ) ) {
		console() << "SUCCEEDED " << getElapsedFrames() << endl;
	}

	if ( SUCCEEDED( hr ) && mDeviceOptions.isBodyEnabled() ) {
		IBodyFrameReference* frameRef = 0;
		hr = frame->get_BodyFrameReference( &frameRef );
		if ( SUCCEEDED( hr ) ) {
			hr = frameRef->AcquireFrame( &bodyFrame );
		}
		if ( frameRef != 0 ) {
			frameRef->Release();
			frameRef = 0;
		}
	}

	if ( SUCCEEDED( hr ) && mDeviceOptions.isBodyIndexEnabled() ) {
		IBodyIndexFrameReference* frameRef = 0;
		hr = frame->get_BodyIndexFrameReference( &frameRef );
		if ( SUCCEEDED( hr ) ) {
			hr = frameRef->AcquireFrame( &bodyIndexFrame );
		}
		if ( frameRef != 0 ) {
			frameRef->Release();
			frameRef = 0;
		}
	}

	if ( SUCCEEDED( hr ) && mDeviceOptions.isColorEnabled() ) {
		IColorFrameReference* frameRef = 0;
		hr = frame->get_ColorFrameReference( &frameRef );
		if ( SUCCEEDED( hr ) ) {
			hr = frameRef->AcquireFrame( &colorFrame );
		}
		if ( frameRef != 0 ) {
			frameRef->Release();
			frameRef = 0;
		}
	}

	if ( SUCCEEDED( hr ) && mDeviceOptions.isDepthEnabled() ) {
		IDepthFrameReference* frameRef = 0;
		hr = frame->get_DepthFrameReference( &frameRef );
		if ( SUCCEEDED( hr ) ) {
			hr = frameRef->AcquireFrame( &depthFrame );
		}
		if ( frameRef != 0 ) {
			frameRef->Release();
			frameRef = 0;
		}
	}

	if ( SUCCEEDED( hr ) && mDeviceOptions.isInfraredEnabled() ) {
		IInfraredFrameReference* frameRef = 0;
		hr = frame->get_InfraredFrameReference( &frameRef );
		if ( SUCCEEDED( hr ) ) {
			hr = frameRef->AcquireFrame( &infraredFrame );
		}
		if ( frameRef != 0 ) {
			frameRef->Release();
			frameRef = 0;
		}
	}

	if ( SUCCEEDED( hr ) && mDeviceOptions.isInfraredLongExposureEnabled() ) {
		ILongExposureInfraredFrameReference* frameRef = 0;
		hr = frame->get_LongExposureInfraredFrameReference( &frameRef );
		if ( SUCCEEDED( hr ) ) {
			hr = frameRef->AcquireFrame( &infraredLongExposureFrame );
		}
		if ( frameRef != 0 ) {
			frameRef->Release();
			frameRef = 0;
		}
	}

	if ( SUCCEEDED( hr ) ) {
		long long time											= 0L;

		// TODO audio

		IFrameDescription* bodyFrameDescription					= 0;
		int32_t bodyWidth										= 0;
		int32_t bodyHeight										= 0;
		uint32_t bodyBufferSize									= 0;
		uint8_t* bodyBuffer										= 0;

		IFrameDescription* bodyIndexFrameDescription			= 0;
		int32_t bodyIndexWidth									= 0;
		int32_t bodyIndexHeight									= 0;
		uint32_t bodyIndexBufferSize							= 0;
		uint8_t* bodyIndexBuffer								= 0;
		
		IFrameDescription* colorFrameDescription				= 0;
		int32_t colorWidth										= 0;
		int32_t colorHeight										= 0;
		ColorImageFormat imageFormat							= ColorImageFormat_None;
		uint32_t colorBufferSize								= 0;
		uint8_t* colorBuffer									= 0;

		IFrameDescription* depthFrameDescription				= 0;
		int32_t depthWidth										= 0;
		int32_t depthHeight										= 0;
		uint16_t depthMinReliableDistance						= 0;
		uint16_t depthMaxReliableDistance						= 0;
		uint32_t depthBufferSize								= 0;
		uint16_t* depthBuffer									= 0;

		IFrameDescription* infraredFrameDescription				= 0;
		int32_t infraredWidth									= 0;
		int32_t infraredHeight									= 0;
		uint32_t infraredBufferSize								= 0;
		uint16_t* infraredBuffer								= 0;

		IFrameDescription* infraredLongExposureFrameDescription	= 0;
		int32_t infraredLongExposureWidth						= 0;
		int32_t infraredLongExposureHeight						= 0;
		uint32_t infraredLongExposureBufferSize					= 0;
		uint16_t* infraredLongExposureBuffer					= 0;

		hr = depthFrame->get_RelativeTime( &time );

		// TODO audio
		if ( mDeviceOptions.isAudioEnabled() ) {

		}

		// TODO body
		if ( mDeviceOptions.isBodyEnabled() ) {

		}

		if ( mDeviceOptions.isBodyIndexEnabled() ) {
			if ( SUCCEEDED( hr ) ) {
				hr = bodyIndexFrame->get_FrameDescription( &bodyIndexFrameDescription );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = bodyIndexFrameDescription->get_Width( &bodyIndexWidth );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = bodyIndexFrameDescription->get_Height( &bodyIndexHeight );
			}
			if ( SUCCEEDED( hr ) ) {
 				//hr = bodyIndexFrame->AccessUnderlyingBuffer( &bodyIndexBufferSize, &bodyIndexBuffer );
			}
		}

		if ( mDeviceOptions.isColorEnabled() ) {
			if ( SUCCEEDED( hr ) ) {
				hr = colorFrame->get_FrameDescription( &colorFrameDescription );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = colorFrameDescription->get_Width( &colorWidth );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = colorFrameDescription->get_Height( &colorHeight );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = colorFrame->get_RawColorImageFormat( &imageFormat );
			}
			if ( SUCCEEDED( hr ) ) {
				bool isAllocated	= false;
				SurfaceChannelOrder channelOrder = SurfaceChannelOrder::BGRA;
				if ( imageFormat == ColorImageFormat_Bgra ) {
					hr = colorFrame->AccessRawUnderlyingBuffer( &colorBufferSize, reinterpret_cast<uint8_t**>( &colorBuffer ) );
					channelOrder = SurfaceChannelOrder::BGRA;
				} else if ( imageFormat == ColorImageFormat_Rgba ) {
					hr = colorFrame->AccessRawUnderlyingBuffer( &colorBufferSize, reinterpret_cast<uint8_t**>( &colorBuffer ) );
					channelOrder = SurfaceChannelOrder::RGBA;
				} else {
					isAllocated = true;
					colorBufferSize = colorWidth * colorHeight * sizeof( uint8_t ) * 4;
					colorBuffer = new uint8_t[ colorBufferSize ];
					hr = colorFrame->CopyConvertedFrameDataToArray( colorBufferSize, reinterpret_cast<uint8_t*>( colorBuffer ), ColorImageFormat_Rgba );
					channelOrder = SurfaceChannelOrder::RGBA;
				}

				if ( SUCCEEDED( hr ) ) {
					colorFrame->get_RelativeTime( &time );
					Surface8u colorSurface = Surface8u( colorBuffer, colorWidth, colorHeight, colorWidth * sizeof( uint8_t ) * 4, channelOrder );
					mFrame.mSurfaceColor = Surface8u( colorWidth, colorHeight, false, channelOrder );
					mFrame.mSurfaceColor.copyFrom( colorSurface, colorSurface.getBounds() );

					console() << "Color\n\twidth: " << colorWidth << "\n\theight: " << colorHeight 
						<< "\n\tbuffer size: " << colorBufferSize << "\n\ttime: " << time << endl;
				}

				if ( isAllocated && colorBuffer != 0 ) {
					delete[] colorBuffer;
					colorBuffer = 0;
				}
			}
		}

		if ( mDeviceOptions.isDepthEnabled() ) {
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->get_FrameDescription( &depthFrameDescription );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrameDescription->get_Width( &depthWidth );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrameDescription->get_Height( &depthHeight );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->get_DepthMinReliableDistance( &depthMinReliableDistance );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->get_DepthMaxReliableDistance( &depthMaxReliableDistance );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = depthFrame->AccessUnderlyingBuffer( &depthBufferSize, &depthBuffer );
			}
			if ( SUCCEEDED( hr ) ) {
				Channel16u depthChannel = Channel16u( depthWidth, depthHeight, depthWidth * sizeof( uint16_t ), 1, depthBuffer );
				mFrame.mChannelDepth = Channel16u( depthWidth, depthHeight );
				mFrame.mChannelDepth.copyFrom( depthChannel, depthChannel.getBounds() );

				console( ) << "Depth\n\twidth: " << depthWidth << "\n\theight: " << depthHeight << endl;
			}
		}

		if ( mDeviceOptions.isInfraredEnabled() ) {
			if ( SUCCEEDED( hr ) ) {
				hr = infraredFrame->get_FrameDescription( &infraredFrameDescription );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = infraredFrameDescription->get_Width( &infraredWidth );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = infraredFrameDescription->get_Height( &infraredHeight );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = infraredFrame->AccessUnderlyingBuffer( &infraredBufferSize, &infraredBuffer );
			}
			if ( SUCCEEDED( hr ) ) {
				Channel16u infraredChannel = Channel16u( infraredWidth, infraredHeight, infraredWidth * sizeof( uint16_t ), 1, infraredBuffer );
				mFrame.mChannelInfrared = Channel16u( infraredWidth, infraredHeight );
				mFrame.mChannelInfrared.copyFrom( infraredChannel, infraredChannel.getBounds() );

				console( ) << "Infrared\n\twidth: " << infraredWidth << "\n\theight: " << infraredHeight << endl;
			}
		}

		if ( mDeviceOptions.isInfraredLongExposureEnabled() ) {
			if ( SUCCEEDED( hr ) ) {
				hr = infraredLongExposureFrame->get_FrameDescription( &infraredLongExposureFrameDescription );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = infraredLongExposureFrameDescription->get_Width( &infraredLongExposureWidth );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = infraredLongExposureFrameDescription->get_Height( &infraredLongExposureHeight );
			}
			if ( SUCCEEDED( hr ) ) {
				hr = infraredLongExposureFrame->AccessUnderlyingBuffer( &infraredLongExposureBufferSize, &infraredLongExposureBuffer );
			}
			if ( SUCCEEDED( hr ) ) {
				Channel16u infraredLongExposureChannel = Channel16u( infraredLongExposureWidth, infraredLongExposureHeight, infraredLongExposureWidth * sizeof( uint16_t ), 1, infraredLongExposureBuffer );
				mFrame.mChannelInfraredLongExposure = Channel16u( infraredLongExposureWidth, infraredLongExposureHeight );
				mFrame.mChannelInfraredLongExposure.copyFrom( infraredLongExposureChannel, infraredLongExposureChannel.getBounds() );

				int64_t irLongExpTime = 0;
				hr = infraredLongExposureFrame->get_RelativeTime( &irLongExpTime );

				console( ) << "Infrared Long Exposure\n\twidth: " << infraredLongExposureWidth << "\n\theight: " << infraredLongExposureHeight;
				if ( SUCCEEDED( hr ) ) {
					console() << "\n\ttimestamp: " << irLongExpTime;
				}
				console() << endl;
			}
		}

		if ( SUCCEEDED( hr ) ) {
			// TODO build Kinect2::Frame from buffers, data
			mFrame.mTimeStamp = time;
		}

		if ( bodyFrameDescription != 0 ) {
			bodyFrameDescription->Release();
			bodyFrameDescription = 0;
		}
		if ( bodyIndexFrameDescription != 0 ) {
			bodyIndexFrameDescription->Release();
			bodyIndexFrameDescription = 0;
		}
		if ( colorFrameDescription != 0 ) {
			colorFrameDescription->Release();
			colorFrameDescription = 0;
		}
		if ( depthFrameDescription != 0 ) {
			depthFrameDescription->Release();
			depthFrameDescription = 0;
		}
		if ( infraredFrameDescription != 0 ) {
			infraredFrameDescription->Release();
			infraredFrameDescription = 0;
		}
		if ( infraredLongExposureFrameDescription != 0 ) {
			infraredLongExposureFrameDescription->Release();
			infraredLongExposureFrameDescription = 0;
		}
	}

	if ( audioFrame != 0 ) {
		audioFrame->Release();
		audioFrame = 0;
	}
	if ( bodyFrame != 0 ) {
		bodyFrame->Release();
		bodyFrame = 0;
	}
	if ( bodyIndexFrame != 0 ) {
		bodyIndexFrame->Release();
		bodyIndexFrame = 0;
	}
	if ( colorFrame != 0 ) {
		colorFrame->Release();
		colorFrame = 0;
	}
	if ( depthFrame != 0 ) {
		depthFrame->Release();
		depthFrame = 0;
	}
	if ( frame != 0 ) {
		frame->Release();
		frame = 0;
	}
	if ( infraredFrame != 0 ) {
		infraredFrame->Release();
		infraredFrame = 0;
	}
	if ( infraredLongExposureFrame != 0 ) {
		infraredLongExposureFrame->Release();
		infraredLongExposureFrame = 0;
	}
}

string Device::wcharToString( wchar_t* v )
{
	string str = "";
	wchar_t* id = ::SysAllocString( v );
	_bstr_t idStr( id );
	if ( idStr.length() > 0 ) {
		str = string( idStr );
	}
	::SysFreeString( id );
	return str;
}

}
