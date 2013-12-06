#include "cinder/app/AppNative.h"
#include "cinder/gl/Texture.h"
#include "cinder/params/Params.h"
#include "cinder/MayaCamUI.h"

#include "Kinect2.h"

class BodyApp : public ci::app::AppBasic
{
public:
	void						draw();
	void						mouseDown( ci::app::MouseEvent event );
	void						mouseDrag( ci::app::MouseEvent event );
	void						prepareSettings( ci::app::AppBasic::Settings* settings );
	void						setup();
	void						update();

private:
	static const uint8_t		kMaxUsers;
	static const ci::Color8u	kUserColors[];

	ci::Color8u					getUserColor( uint8_t bodyIndex ) const;

	ci::MayaCamUI				mMayaCam;

	ci::gl::TextureRef			mTextureColor;
	ci::gl::TextureRef			mTextureDepth;
	ci::gl::TextureRef			mTextureBodyIndex;
	Kinect2::DeviceRef			mDevice;

	ci::Font					mFont;

	ci::params::InterfaceGlRef	mParams;
	float						mFrameRate;
	bool						mIsFullscreen;
};

#include "cinder/gl/gl.h"
#include "cinder/Utilities.h"

using namespace ci;
using namespace ci::app;
using namespace std;

const uint8_t BodyApp::kMaxUsers = 6;
const ci::Color8u BodyApp::kUserColors[ BodyApp::kMaxUsers ] = {
	Color8u( 255, 0, 0 ),
	Color8u( 255, 255, 0 ),
	Color8u( 255, 0, 255 ),
	Color8u( 0, 255, 255 ),
	Color8u( 0, 255, 0 ),
	Color8u( 0, 0, 255 )
};

Color8u BodyApp::getUserColor( uint8_t bodyIndex ) const
{
	if ( bodyIndex < kMaxUsers ) {
		return kUserColors[ bodyIndex ];
	}

	return Color8u::white();
}

void BodyApp::draw()
{
	// draw 3D at bottom right quadrant
	Area viewport = Area( getWindowCenter().x, 0, getWindowWidth(), getWindowCenter().y );

	gl::setViewport( viewport );
	gl::clear( Colorf::black() );
	gl::setMatrices( mMayaCam.getCamera() );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableWireframe();

	if ( mDevice ) {
		const vector<Kinect2::User>& users = mDevice->getFrame().getUsers();
		for ( const Kinect2::User& user : users ) {
			gl::color( kUserColors[ user.getBodyIndex() ] );
			for ( auto jointPair : user.getJointMap() ) {
				gl::pushModelView();
				gl::translate( jointPair.second.mPosition );
				gl::rotate( jointPair.second.mOrientation );
				gl::scale( 0.05f, 0.05f, 0.05f );
				gl::drawCube( Vec3f::zero(), Vec3f::one() );
				gl::popModelView();
			}
		}
	}

	gl::disableWireframe();
	gl::disableDepthRead();
	gl::disableDepthWrite();

	// draw color in top left quadrant
	viewport = Area( 0, getWindowCenter().y, getWindowCenter().x, getWindowHeight() );

	gl::setViewport( viewport );
	gl::setMatricesWindow( viewport.getSize() );
	gl::color( Colorf::white() );

	if ( mTextureColor ) {
		gl::draw( mTextureColor, mTextureColor->getBounds(), Rectf( Vec2f::zero(), viewport.getSize() ) );
		mTextureColor->unbind();
		
		// draw joints
		if ( mDevice ) {
			Vec2f s = Vec2f( viewport.getSize() ) / Vec2f( mTextureColor->getSize() );

			const vector<Kinect2::User>& users = mDevice->getFrame().getUsers();
			for ( const Kinect2::User& user : users ) {
				gl::color( kUserColors[ user.getBodyIndex() ] );
				for ( auto jointPair : user.getJointMap() ) {
					Vec2f colorPos = mDevice->getJointPositionInColorFrame( jointPair.second.mPosition );
					colorPos *= s;

					//console( ) << "scale: " << s << "\n" << "position: " << colorPos << endl;

					if ( jointPair.first == JointType_SpineBase ) {
						gl::drawString( "User Id: " + toString( (int32_t)user.getBodyIndex() ), colorPos + Vec2f( 8.0f, 0.0f ), ColorAf( 0.0f, 1.0f, 1.0f ), mFont );
					}

					gl::drawSolidCircle( colorPos, 5.0f );
				}
			}
		}
	}

	// draw depth in top right quadrant
	viewport = Area( Vec2i( getWindowCenter() ), getWindowSize() );

	gl::setViewport( viewport );
	gl::setMatricesWindow( viewport.getSize() );
	gl::color( Colorf::white() );

	if ( mTextureDepth ) {
		gl::draw( mTextureDepth, mTextureDepth->getCleanBounds(), Rectf( Vec2f::zero(), viewport.getSize() ) );
	}

	// draw body index in bottom left
	viewport = Area( Vec2i::zero(), Vec2i( getWindowCenter() ) );

	gl::setViewport( viewport );
	gl::setMatricesWindow( viewport.getSize() );
	gl::color( Colorf::white() );

	if ( mTextureBodyIndex ) {
		gl::draw( mTextureBodyIndex, mTextureBodyIndex->getCleanBounds(), Rectf( Vec2f::zero(), viewport.getSize() ) );
	}

	mParams->draw();
}

void BodyApp::mouseDown( MouseEvent event )
{
	mMayaCam.mouseDown( event.getPos() );
}

void BodyApp::mouseDrag( MouseEvent event )
{
	mMayaCam.mouseDrag( event.getPos(), event.isLeftDown(), event.isMiddleDown(), event.isRightDown() );
}

void BodyApp::prepareSettings( Settings* settings )
{
	settings->prepareWindow( Window::Format().size( 1280, 720 ).title( "Body App" ) );
	settings->setFrameRate( 60.0f );
}

void BodyApp::setup()
{
	mFont = Font( "Arial", 32.0f );

	// setup camera
	CameraPersp cam = mMayaCam.getCamera();
	cam.setFov( 53.8f );
	cam.setEyePoint( Vec3f::zero() );
	cam.setCenterOfInterestPoint( Vec3f::zAxis() * 5.0f );
	mMayaCam.setCurrentCam( cam );

	// setup Kinect
	mDevice = Kinect2::Device::create();
	mDevice->start( Kinect2::DeviceOptions().enableBody().enableBodyIndex() );

	// setup params
	mFrameRate = 0.0f;
	mIsFullscreen = isFullScreen();
	mParams = params::InterfaceGl::create( "PARAMS", Vec2i( 180, 200 ) );

	mParams->addParam( "Frame rate", &mFrameRate, "", true );
	mParams->addParam( "Fullscreen", &mIsFullscreen, "key=f" );
	mParams->addButton( "Quit", bind( &BodyApp::quit, this ), "key=q" );

}

void BodyApp::update()
{
	mFrameRate = getAverageFps();

	if ( mIsFullscreen != isFullScreen() ) {
		setFullScreen( mIsFullscreen );
	}

	if ( mDevice ) {
		Surface8u color				= mDevice->getFrame().getColor();
		Channel16u depth16u			= mDevice->getFrame().getDepth();
		Channel8u depth8u			= Channel8u( depth16u.getWidth(), depth16u.getHeight() );
		Channel8u bodyIndexChan		= mDevice->getFrame().getBodyIndex();
		Surface8u bodyIndexSurf		= Surface8u( bodyIndexChan.getWidth(), bodyIndexChan.getHeight(), false, SurfaceChannelOrder::RGBA );

		Surface8u::Iter colorIt			= color.getIter();
		Channel16u::Iter depth16It		= depth16u.getIter();
		Channel8u::Iter depth8It		= depth8u.getIter();
		Channel8u::Iter bodyIndexChanIt = bodyIndexChan.getIter();
		Surface8u::Iter bodyIndexSurfIt = bodyIndexSurf.getIter();

		while ( depth16It.line() && depth8It.line() && bodyIndexChanIt.line() && bodyIndexSurfIt.line() ) {
			while ( depth16It.pixel() && depth8It.pixel() && bodyIndexChanIt.pixel() && bodyIndexSurfIt.pixel() ) {
				depth8It.v() = depth16It.v() % 256;

				size_t userColorIndex = bodyIndexChanIt.v() % kMaxUsers;
				if ( bodyIndexChanIt.v() < 0xff ) {
					bodyIndexSurfIt.r() = kUserColors[ userColorIndex ].r;
					bodyIndexSurfIt.g() = kUserColors[ userColorIndex ].g;
					bodyIndexSurfIt.b() = kUserColors[ userColorIndex ].b;
					bodyIndexSurfIt.a() = 255;
				} else {
					bodyIndexSurfIt.r() = 0;
					bodyIndexSurfIt.g() = 0;
					bodyIndexSurfIt.b() = 0;
					bodyIndexSurfIt.a() = 255;
				}
			}
		}

		if ( !mTextureColor ) {
			mTextureColor = gl::Texture::create( color );
		} else {
			mTextureColor->update( color );
		}
		
		if ( !mTextureDepth ) {
			mTextureDepth = gl::Texture::create( depth8u );
		} else {
			mTextureDepth->update( depth8u, depth8u.getBounds() );
		}

		if ( !mTextureBodyIndex ) {
			mTextureBodyIndex = gl::Texture::create( bodyIndexSurf );
		} else {
			mTextureBodyIndex->update( bodyIndexSurf );
		}
	}
}

CINDER_APP_NATIVE( BodyApp, RendererGl )
