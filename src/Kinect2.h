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

#pragma once

#include "cinder/Exception.h"
#include "cinder/Matrix.h"
#include "cinder/Quaternion.h"
#include "cinder/Surface.h"
#include <functional>
#include <map>
#include "ole2.h"

#if defined( _DEBUG )
#pragma comment( lib, "comsuppwd.lib" )
#else
#pragma comment( lib, "comsuppw.lib" )
#endif
#pragma comment( lib, "wbemuuid.lib" )

#include "Kinect.h"

namespace Kinect2 {

std::string getStatusMessage( KinectStatus status );

//////////////////////////////////////////////////////////////////////////////////////////////

class DeviceOptions
{
public:
	DeviceOptions();
	
	DeviceOptions&						enableAudio( bool enable = true );
	DeviceOptions&						enableBody( bool enable = true );
	DeviceOptions&						enableBodyIndex( bool enable = true );
	DeviceOptions&						enableColor( bool enable = true );
	DeviceOptions&						enableDepth( bool enable = true );
	DeviceOptions&						enableInfrared( bool enable = true );
	DeviceOptions&						enableInfraredLongExposure( bool enable = true );
	DeviceOptions&						setDeviceId( const std::string& id = "" ); 
	DeviceOptions&						setDeviceIndex( int32_t index = 0 );

	const std::string&					getDeviceId() const;
	int32_t								getDeviceIndex() const;
	bool								isAudioEnabled() const;
	bool								isBodyEnabled() const;
	bool								isBodyIndexEnabled() const;
	bool								isColorEnabled() const;
	bool								isDepthEnabled() const;
	bool								isInfraredEnabled() const;
	bool								isInfraredLongExposureEnabled() const;
protected:
	std::string							mDeviceId;
	int32_t								mDeviceIndex;

	bool								mEnabledAudio;
	bool								mEnabledBody;
	bool								mEnabledBodyIndex;
	bool								mEnabledColor;
	bool								mEnabledDepth;
	bool								mEnabledInfrared;
	bool								mEnabledInfraredLongExposure;
};

//////////////////////////////////////////////////////////////////////////////////////////////

class Device;

class User
{
public:
	struct Joint
	{
		ci::Vec3f		mPosition;
		ci::Quatf		mOrientation;
		TrackingState	mTrackingState;
	};

	std::map<JointType, User::Joint>&			getJointMap() { return mJointMap; }
	const std::map<JointType, User::Joint>&		getJointMap() const { return mJointMap; }

	uint64_t									getTrackingId() const { return mTrackingId; }
	bool										getIsTracked() const { return mIsTracked; }

private:
	std::map<JointType, User::Joint>			mJointMap;
	bool										mIsTracked;
	uint64_t									mTrackingId;

	friend class Device;
};

class Frame
{
public:
	Frame();

	const ci::Surface8u&				getColor() const;
	const ci::Channel16u&				getDepth() const;
	const std::string&					getDeviceId() const;
	const ci::Channel16u&				getInfrared() const;
	const ci::Channel16u&				getInfraredLongExposure() const;
	long long							getTimeStamp() const;

	const std::vector<User>&			getUsers() const { return mUsers; }
protected:
	Frame( long long frameId, const std::string& deviceId, const ci::Surface8u& color, 
		const ci::Channel16u& depth, const ci::Channel16u& infrared, 
		const ci::Channel16u& infraredLongExposure );

	std::string							mDeviceId;
	ci::Channel16u						mChannelDepth;
	ci::Channel16u						mChannelInfrared;
	ci::Channel16u						mChannelInfraredLongExposure;
	ci::Surface8u						mSurfaceColor;
	long long							mTimeStamp;

	std::vector<User>					mUsers;

	friend class						Device;
};

//////////////////////////////////////////////////////////////////////////////////////////////

typedef std::shared_ptr<Device>	DeviceRef;

class Device
{
public:
	static DeviceRef					create();
	~Device();
	
	void								start( const DeviceOptions& deviceOptions = DeviceOptions() );
	void								stop();

	const DeviceOptions&				getDeviceOptions() const;
	const Frame&						getFrame() const;
	KinectStatus						getStatus() const;

	// mapping methods
	ci::Vec2i							getJointPositionInColorFrame( const ci::Vec3f& jointPosition ) const;
	ci::Vec2i							getJointPositionInDepthFrame( const ci::Vec3f& jointPosition ) const;
protected:
	Device();

	virtual void						update();

	std::function<void ( Frame frame )>	mEventHandler;
	
	ICoordinateMapper*					mCoordinateMapper;
	IMultiSourceFrameReader*			mFrameReader;
	IKinectSensor*						mSensor;
	//WAITABLE_HANDLE					onSensorCollectionChanged();

	DeviceOptions						mDeviceOptions;
	Frame								mFrame;
	KinectStatus						mStatus;
	std::string							mStatusMessage();

	std::string							wcharToString( wchar_t* v );
};

}
