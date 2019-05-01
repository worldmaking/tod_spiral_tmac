#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){

	// CONTROL SCREEN:
	ofGLFWWindowSettings settings;
    settings.setSize(1920/4, 1200/4);
	//settings.windowMode = OF_FULLSCREEN;
	//settings.monitor = 0;
	settings.setPosition(ofVec2f(100, 100));
	settings.resizable = true;
	shared_ptr<ofAppBaseWindow> window0 = ofCreateWindow(settings);

	// FIRST SCREEN:
    settings.setSize(1920/4, 1200/4);
	//settings.monitor = 0;
	settings.setPosition(ofVec2f(500, 100));
	settings.resizable = true;
	//settings.shareContextWith = window0;
	shared_ptr<ofAppBaseWindow> window1 = ofCreateWindow(settings);

	// SECOND SCREEN:
	//settings.monitor = 1;
    settings.setSize(1920/4, 1200/4);
	settings.setPosition(ofVec2f(900, 100));
	// share OpenGL resources between windows:
	//settings.shareContextWith = window0;	
	shared_ptr<ofAppBaseWindow> window2 = ofCreateWindow(settings);
	// TODO: yes or no???
	window1->setVerticalSync(false);

	shared_ptr<ofApp> app(new ofApp);
	shared_ptr<RenderApp> app1(new RenderApp);
	shared_ptr<RenderApp> app2(new RenderApp);

	app->app1 = app1;
	app->app2 = app2;
	app->window1 = window1;
	app->window2 = window2;


	ofRunApp(window0, app);
	ofRunApp(window1, app1);
	ofRunApp(window2, app2);
	ofRunMainLoop();
}
