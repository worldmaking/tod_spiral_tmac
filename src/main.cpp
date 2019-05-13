#include "ofMain.h"
#include "ofApp.h"
#include "ofRenderApp.h"

Simulation simulation;

Simulation& Simulation::get() {
	return simulation;
}

//========================================================================
int main( ){

	// CONTROL SCREEN:
	ofGLFWWindowSettings settings;
    settings.setSize(1920/2, 1200/2);
	settings.setGLVersion(3,2);
	//settings.windowMode = OF_FULLSCREEN;
	//settings.monitor = 0;
	settings.setPosition(ofVec2f(0, 60));
	settings.resizable = true;
	shared_ptr<ofAppBaseWindow> window0 = ofCreateWindow(settings);

	// FIRST SCREEN:
    settings.setSize(1920/4, 1200/4);
	//settings.monitor = 1;
	settings.setPosition(ofVec2f(960, 60));
	settings.resizable = true;
	settings.shareContextWith = window0;
	shared_ptr<ofAppBaseWindow> window1 = ofCreateWindow(settings);

	// SECOND SCREEN:
	//settings.monitor = 2;
    settings.setSize(1920/4, 1200/4);
	settings.setPosition(ofVec2f(1440, 60));
	// share OpenGL resources between windows:
	settings.shareContextWith = window0;	
	shared_ptr<ofAppBaseWindow> window2 = ofCreateWindow(settings);
	// TODO: yes or no???
	window1->setVerticalSync(false);

	shared_ptr<ofApp> app(new ofApp);
	shared_ptr<RenderApp> app1(new RenderApp(app));
	shared_ptr<RenderApp> app2(new RenderApp(app));


	ofRunApp(window0, app);
	ofRunApp(window1, app1);
	ofRunApp(window2, app2);
	ofRunMainLoop();
}
