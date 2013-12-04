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

#include "cinder/app/AppBasic.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"

#include "Kinect2.h"

class BasicApp : public ci::app::AppBasic 
{
public:
	void						draw();
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();
private:
	ci::gl::TextureRef			mTextureColor;
	ci::gl::TextureRef			mTextureDepth;
	ci::gl::TextureRef			mTextureInfrared;
	ci::gl::TextureRef			mTextureInfraredLongExposure;

	Kinect2::DeviceRef			mDevice;

	float						mFrameRate;
	bool						mFullScreen;
	ci::params::InterfaceGlRef	mParams;
};

#include "cinder/Font.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void BasicApp::draw()
{
	gl::setViewport( getWindowBounds() );
	gl::clear( Colorf::white() );
	gl::setMatricesWindow( getWindowSize() );
	
	if ( mTextureColor ) {
		gl::draw( mTextureColor, mTextureColor->getBounds(), Rectf( Vec2f::zero(), getWindowCenter() ) );
		mTextureColor->unbind();
	}

	if ( mTextureDepth ) {
		gl::draw( mTextureDepth, mTextureDepth->getBounds(), Rectf( getWindowCenter().x, 0.0f, getWindowWidth(), getWindowCenter().y ) );
		mTextureDepth->unbind();
	}

	if ( mTextureInfrared ) {
		gl::draw( mTextureInfrared, mTextureInfrared->getBounds(), Rectf( 0.0f, getWindowCenter().y, getWindowCenter().x, getWindowHeight() ) );
		mTextureInfrared->unbind();
	}

	if ( mTextureInfraredLongExposure ) {
		gl::draw( mTextureInfraredLongExposure, mTextureInfraredLongExposure->getBounds(), Rectf( getWindowCenter(), getWindowSize() ) );
		mTextureInfraredLongExposure->unbind();
	}

	mParams->draw();
}

void BasicApp::prepareSettings( Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 1280, 720 ).title( "Basic App" ) );
	settings->setFrameRate( 60.0f );
}

void BasicApp::setup()
{	
	gl::enable( GL_TEXTURE_2D );
	
	mFrameRate	= 0.0f;
	mFullScreen	= false;

	mDevice = Kinect2::Device::create();
	mDevice->start( Kinect2::DeviceOptions().enableInfrared().enableInfraredLongExposure() );
			
	mParams = params::InterfaceGl::create( "Params", Vec2i( 200, 150 ) );
	mParams->addParam( "Frame rate",	&mFrameRate,				"", true );
	mParams->addParam( "Full screen",	&mFullScreen,				"key=f" );
	mParams->addButton( "Quit", bind(	&BasicApp::quit, this ),	"key=q" );
}

void BasicApp::update()
{
	mFrameRate = getAverageFps();
	
	if ( mFullScreen != isFullScreen() ) {
		setFullScreen( mFullScreen );
		mFullScreen = isFullScreen();
	}

	if ( mDevice ) {
		if ( mDevice->getDeviceOptions().isColorEnabled() ) {
			mTextureColor = gl::Texture::create( mDevice->getFrame().getColor() );
		}
		
		if ( mDevice->getDeviceOptions().isDepthEnabled() ) {
			const Channel16u& depthChannel16	= mDevice->getFrame().getDepth();
			Channel8u depthChannel8				= Channel8u( depthChannel16.getWidth(), depthChannel16.getHeight() );
			Channel16u::ConstIter depthIter16	= depthChannel16.getIter();
			Channel8u::Iter depthIter8			= depthChannel8.getIter();

			while ( depthIter8.line() && depthIter16.line() ) {
				while ( depthIter8.pixel() && depthIter16.pixel() ) {
					depthIter8.v() = depthIter16.v() % 256;
				}
			}

			mTextureDepth = gl::Texture::create( depthChannel8 );
		}

		if ( mDevice->getDeviceOptions().isInfraredEnabled() ) {
			const Channel16u& infraredChannel16		= mDevice->getFrame().getInfrared();
			Channel8u infraredChannel8				= Channel8u( infraredChannel16.getWidth(), infraredChannel16.getHeight() );
			Channel16u::ConstIter infraredIter16	= infraredChannel16.getIter();
			Channel8u::Iter infraredIter8			= infraredChannel8.getIter();

			while ( infraredIter8.line() && infraredIter16.line() ) {
				while ( infraredIter8.pixel() && infraredIter16.pixel() ) {
					infraredIter8.v() = infraredIter16.v() >> 8;
				}
			}

			mTextureInfrared = gl::Texture::create( infraredChannel8 );
		}

		if ( mDevice->getDeviceOptions().isInfraredLongExposureEnabled() ) {
			//const Channel16u& infraredChannel16		= mDevice->getFrame().getInfraredLongExposure();
			//Channel8u infraredChannel8				= Channel8u( infraredChannel16.getWidth(), infraredChannel16.getHeight() );
			//Channel16u::ConstIter infraredIter16	= infraredChannel16.getIter();
			//Channel8u::Iter infraredIter8			= infraredChannel8.getIter();

			//while ( infraredIter8.line() && infraredIter16.line() ) {
			//	while ( infraredIter8.pixel() && infraredIter16.pixel() ) {
			//		infraredIter8.v() = infraredIter16.v() >> 8;
			//	}
			//}

			mTextureInfraredLongExposure = gl::Texture::create( Channel8u( mDevice->getFrame().getInfraredLongExposure() ) );
		}
	}
}

CINDER_APP_BASIC( BasicApp, RendererGl )
	