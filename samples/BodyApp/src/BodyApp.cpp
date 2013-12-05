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
	ci::MayaCamUI				mMayaCam;

	ci::gl::TextureRef			mTextureColor;
	Kinect2::DeviceRef			mDevice;

	ci::params::InterfaceGlRef	mParams;
	float						mFrameRate;
	bool						mIsFullscreen;
};

#include "cinder/gl/gl.h"

using namespace ci;
using namespace ci::app;
using namespace std;

void BodyApp::draw()
{
	gl::setViewport( Area( Vec2i( getWindowCenter().x, 0 ), getWindowSize() ) );
	gl::clear( Colorf::black() );
	gl::setMatrices( mMayaCam.getCamera() );
	gl::enableDepthRead();
	gl::enableDepthWrite();
	gl::enableWireframe();

	if ( mDevice ) {
		gl::color( Colorf( 1.0f, 0.0f, 1.0f ) );
		const vector<Kinect2::User>& users = mDevice->getFrame().getUsers();
		for ( const Kinect2::User& user : users ) {
			for ( auto jointPair : user.getJointMap() ) {
				gl::pushModelView();
				gl::translate( jointPair.second.mPosition );
				gl::rotate( jointPair.second.mOrientation );
				gl::drawSphere( Vec3f::zero(), 0.01f );
				gl::popModelView();
			}
		}
	}

	gl::disableWireframe();
	gl::disableDepthRead();
	gl::disableDepthWrite();

	Area viewport = Area( Vec2i::zero(), Vec2i( getWindowCenter().x, getWindowHeight() ) );
	gl::setViewport( viewport );
	gl::setMatricesWindow( viewport.getSize() );
	gl::color( Colorf::white() );

	if ( mTextureColor ) {
		const float w = 240.0f;
		const float h = 135.0f;
		gl::draw( mTextureColor, mTextureColor->getBounds(), Rectf( Vec2f::zero(), viewport.getSize() ) );
		mTextureColor->unbind();
		
		if ( mDevice ) {
			Vec2f s = Vec2f( viewport.getSize() ) / Vec2f( mTextureColor->getSize() );

			gl::color( Colorf( 1.0f, 0.0f, 1.0f ) );
			const vector<Kinect2::User>& users = mDevice->getFrame().getUsers();
			for ( const Kinect2::User& user : users ) {
				for ( auto jointPair : user.getJointMap() ) {
					Vec2f colorPos = mDevice->getJointPositionInColorFrame( jointPair.second.mPosition );
					
					console( ) << "scale: " << s << "\n" << "position: " << colorPos << endl;

					colorPos *= s;
					gl::drawSolidCircle( colorPos, 5.0f );
				}
			}
		}
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
	// setup camera
	CameraPersp cam = mMayaCam.getCamera();
	cam.setFov( 53.8f );
	cam.setEyePoint( Vec3f::zero() );
	cam.setCenterOfInterestPoint( Vec3f::zAxis() * 5.0f );
	mMayaCam.setCurrentCam( cam );

	// setup Kinect
	mDevice = Kinect2::Device::create();
	mDevice->start( Kinect2::DeviceOptions().enableBody() );

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
		mTextureColor = gl::Texture::create( mDevice->getFrame().getColor() );
	}
}

CINDER_APP_NATIVE( BodyApp, RendererGl )
