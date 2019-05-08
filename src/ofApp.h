#pragma once

#include "ofMain.h"
#include "ofxOpenVR.h"
#include "of3dGraphics.h"

#include "al/al_kinect2.h"

void ThreadSleep(unsigned long nMilliseconds)
{
#if defined(_WIN32)
	::Sleep(nMilliseconds);
#elif defined(POSIX)
	usleep(nMilliseconds * 1000);
#endif
}

glm::mat4 ConvertSteamVRMatrixToMatrix4(const vr::HmdMatrix34_t &matPose)
{
	glm::mat4 matrixObj(
		matPose.m[0][0], matPose.m[1][0], matPose.m[2][0], 0.0,
		matPose.m[0][1], matPose.m[1][1], matPose.m[2][1], 0.0,
		matPose.m[0][2], matPose.m[1][2], matPose.m[2][2], 0.0,
		matPose.m[0][3], matPose.m[1][3], matPose.m[2][3], 1.0f
	);
	return matrixObj;
}

std::string GetTrackedDeviceString(vr::TrackedDeviceIndex_t unDevice, vr::TrackedDeviceProperty prop, vr::TrackedPropertyError *peError = NULL)
{
	uint32_t unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
	if (unRequiredBufferLen == 0)
		return "";

	char *pchBuffer = new char[unRequiredBufferLen];
	unRequiredBufferLen = vr::VRSystem()->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
	std::string sResult = pchBuffer;
	delete[] pchBuffer;
	return sResult;
}

class ofApp : public ofBaseApp {

public:
	bool isFullscreen = false;
	bool bShowHelp = false;

	CloudDeviceManager cloudDeviceManager;

	// vector to store all values
	vector <ofVec3f> points;
	vector <ofVec3f> sizes;

	ofxOpenVR openVR;

	bool bUseShader;

	float polylineResolution = .004f;

	vector<ofPolyline> leftControllerPolylines;
	vector<ofPolyline> rightControllerPolylines;
	bool bIsLeftTriggerPressed = false;
	bool bIsRightTriggerPressed = false;
	ofVec3f leftControllerPosition;
	ofVec3f rightControllerPosition;
	ofVec3f lastLeftControllerPosition;
	ofVec3f lastRightControllerPosition;
	std::vector< CGLRenderModel * > m_vecRenderModels;
	//CGLRenderModel *FindOrLoadRenderModel(const char *pchRenderModelName);

	struct ControllerInfo_t
	{
		vr::VRInputValueHandle_t m_source = vr::k_ulInvalidInputValueHandle;
		vr::VRActionHandle_t m_actionPose = vr::k_ulInvalidActionHandle;
		vr::VRActionHandle_t m_actionHaptic = vr::k_ulInvalidActionHandle;
		glm::mat4 m_rmat4Pose;
		CGLRenderModel *m_pRenderModel = nullptr;
		std::string m_sRenderModelName;
		bool m_bShowController = true;
	};

	ControllerInfo_t m_rHand[2];
	GLint m_nRenderModelMatrixLocation;
	GLuint m_unRenderModelProgramID;

	ofShader controlShader;

	std::ostringstream _strHelp;

	ofVbo vbo;
	ofShader shaderP;
	ofTexture texture;

	ofTexture kinectTexture[2];
	ofShader shader;

	CloudFrame Cloud;

	bool firstFrameFlag = false; //unused but not removed
	bool pointsCreated = false; //unused but not removed

	bool isCapturing;

	bool rightDown = false;
	bool leftDown = false;

	glm::mat4x4 controllerBegin;
	glm::mat4x4 kinectBegin;

	ofVec3f origin = ofVec3f(-1.f, 0.f, -1.f); //starts the kinect space closer to the kinect (The physical kniect is somewhere < -1,0,-1 in the VR space)
	//ofVec3f cloudMan = ofVec3f(0.f, 0.f, 0.f);

	glm::vec3 kinectPosition[2];
	glm::vec3 controllerDragStartPosition;
	

	void setup() {
		isFullscreen = 0;

		if(1){
			//Sets up a number of particles at position 0 before the kinect starts rendering
			CloudDevice& kinect = cloudDeviceManager.devices[0];
			const CloudFrame& cloud = kinect.cloudFrame();
			const ColourFrame& colour = kinect.colourFrame();
			int num = cDepthWidth * cDepthHeight;
			for (int j = 0; j < num; j++) {
				ofVec3f p;
				p.x = 0;
				p.y = 0;
				p.z = 0;
				addPoint(p.x, p.y, p.z);
			}
		}
		
		// load the texure
		ofDisableArbTex();
		ofLoadImage(texture, "dot.png");

		ofSetFrameRate(90);  // for VR
		ofSetVerticalSync(false); // for VR
		// We need to pass the method we want ofxOpenVR to call when rending the scene
		openVR.setup(std::bind(&ofApp::render, this, std::placeholders::_1));
		openVR.setDrawControllers(true);
		openVR.setRenderModelForTrackedDevices(true);
		ofAddListener(openVR.ofxOpenVRControllerEvent, this, &ofApp::controllerEvent);

		m_nRenderModelMatrixLocation = glGetUniformLocation(m_unRenderModelProgramID, "matrix");

		shaderP.load("shaders/shader"); 

		lastLeftControllerPosition.set(ofVec3f());
		lastRightControllerPosition.set(ofVec3f());

		controlShader.load("shaders_gl3/controller");

		kinectTexture[0].allocate(cColorWidth, cColorHeight, GL_RGBA);
		kinectTexture[1].allocate(cColorWidth, cColorHeight, GL_RGBA);

		cloudDeviceManager.reset();
		cloudDeviceManager.open_all();

		// upload the data to the vbo
		int total = (int)points.size();
		vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
		vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
		shader.load("shaders_gl3/point");
	}

	void addPoint(float x, float y, float z) {
		ofVec3f p(x, y, z);
		points.push_back(p);

		// we are passing the size in as a normal x position
		float size = ofRandom(50, 100);
		sizes.push_back(ofVec3f(size));
	}

	void updatePoints(vector <ofVec3f> pUpdate, vector <ofVec3f> sUpdate) {
		points = pUpdate;
		sizes = sUpdate;
	}

	void exit() {

		for (std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++)
		{
			delete (*i);
		}
		m_vecRenderModels.clear();

		ofRemoveListener(openVR.ofxOpenVRControllerEvent, this, &ofApp::controllerEvent);
		openVR.exit();
	}

	void update() {

		std::stringstream strm;
		strm << "fps: " << ofGetFrameRate();
		ofSetWindowTitle(strm.str());

		
		for (int i = 0; i < 1; i++) { //set i = 2 for two kinects
			CloudDevice& kinect = cloudDeviceManager.devices[i];
			if (kinect.capturing) {

				isCapturing = true; 
				// get most recent frames:
				const CloudFrame& cloud = kinect.cloudFrame();
				const ColourFrame& colour = kinect.colourFrame();

				int num = cDepthWidth * cDepthHeight;
				float radius = 1;

				//glm::mat4 cloudTransform = glm::mat4(1.); this is what kinect2 auto sets the cloud transform to be
				if (bIsLeftTriggerPressed) {

					glm::vec3 controllerTranslation = glm::vec3(openVR.getControllerPose(vr::TrackedControllerRole_LeftHand)[3]); 

					if (!leftDown) {
						// just started dragging:
						leftDown = true;
						controllerDragStartPosition = controllerTranslation;
					}
					else {
						// we are already dragging
						glm::vec3 delta = controllerTranslation - controllerDragStartPosition;
						controllerDragStartPosition = controllerTranslation;

						// update our cloud:
						kinectPosition[i] += delta;
					}

					kinect.cloudTransform = glm::translate(glm::mat4(1.), kinectPosition[i]);

					//printf("moved to %s\n", glm::to_string(kinect.cloudTransform).data());

				}
				else leftDown = false;

				if (bIsRightTriggerPressed) {
					
					glm::vec3 controllerTranslation = glm::vec3(openVR.getControllerPose(vr::TrackedControllerRole_RightHand)[3]);

					if (!rightDown) {
						// just started dragging:
						rightDown = true;
						controllerDragStartPosition = controllerTranslation;
					}
					else {
						// we are already dragging
						glm::vec3 delta = controllerTranslation - controllerDragStartPosition;
						controllerDragStartPosition = controllerTranslation;

						// update our cloud (*3 so the right controller is quicker):
						kinectPosition[i] += (delta * 3);
					}

					kinect.cloudTransform = glm::translate(glm::mat4(1.), kinectPosition[i]);

					//printf("moved to %s\n", glm::to_string(kinect.cloudTransform).data());
				}
				else rightDown = false;

				vector <ofVec3f> pointsUpdate;
				vector <ofVec3f> sizesUpdate;
				for (int j = 0; j < num; j++) {
					ofVec3f p;
					//y and z values must be negative, otherwise kinect particles will be upside-down and reversed
					p.x = cloud.xyz[j].x + origin.x;
					p.y = cloud.xyz[j].y + origin.y;
					p.z = cloud.xyz[j].z + origin.z;
					//if(p.x != 0.f || p.y != 0.f || p.z != 0.f)
					pointsUpdate.push_back(p);
					float size = ofRandom(50, 100);
					sizesUpdate.push_back(ofVec3f(size));
				}
				ofVec3f ref1 = ofVec3f(-1.f, 0.f, -1.f);
				ofVec3f ref2 = ofVec3f(0.f, 0.f, 0.f);
				ofVec3f ref3 = ofVec3f(1.f, 0.f, 1.f);
				
				pointsUpdate.push_back(ref1); //reference point (closest to kinect)
				pointsUpdate.push_back(ref2); //reference point
				pointsUpdate.push_back(ref3); //reference point
				updatePoints(pointsUpdate, sizesUpdate);	//TODO: Reposition and re-orient the kinect space in the VR space. A L G O R I T H M S
				//printf("Points Updated \n");
				//printf("Point 0 of points: %f, %f, %f\n", points[0].x, points[0].y, points[0].z);
				int total = (int)points.size();
				vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
				vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
			}
		}

		if (isFullscreen){
			ofHideCursor();
		} else {
		 	ofShowCursor();
		}
		openVR.update();

		for (int eHand = 0; eHand <= 1; eHand++)
		{
			vr::InputPoseActionData_t poseData;
			vr::EVRInputError err = vr::VRInput()->GetPoseActionData(m_rHand[eHand].m_actionPose, vr::TrackingUniverseStanding, 0, &poseData, sizeof(poseData), vr::k_ulInvalidInputValueHandle);
			if (err != vr::VRInputError_None
				|| !poseData.bActive || !poseData.pose.bPoseIsValid)
			{
				//printf("input %d %d %d %d\n", eHand, err, !poseData.bActive , !poseData.pose.bPoseIsValid);
				m_rHand[eHand].m_bShowController = false;
			}
			else
			{
				m_rHand[eHand].m_rmat4Pose = ConvertSteamVRMatrixToMatrix4(poseData.pose.mDeviceToAbsoluteTracking);

				vr::InputOriginInfo_t originInfo;
				if (vr::VRInput()->GetOriginTrackedDeviceInfo(poseData.activeOrigin, &originInfo, sizeof(originInfo)) == vr::VRInputError_None
					&& originInfo.trackedDeviceIndex != vr::k_unTrackedDeviceIndexInvalid)
				{
					std::string sRenderModelName = GetTrackedDeviceString(originInfo.trackedDeviceIndex, vr::Prop_RenderModelName_String);
					if (sRenderModelName != m_rHand[eHand].m_sRenderModelName)
					{
						m_rHand[eHand].m_pRenderModel = FindOrLoadRenderModel(sRenderModelName.c_str());
						m_rHand[eHand].m_sRenderModelName = sRenderModelName;
					}
				}
			}
		}

		if (bIsLeftTriggerPressed) {
			if (openVR.isControllerConnected(vr::TrackedControllerRole_LeftHand)) {
				// Getting the translation component of the controller pose matrix
				leftControllerPosition = openVR.getControllerPose(vr::TrackedControllerRole_LeftHand)[3];

				if (lastLeftControllerPosition.distance(leftControllerPosition) >= polylineResolution) {
					leftControllerPolylines[leftControllerPolylines.size() - 1].lineTo(leftControllerPosition);
					lastLeftControllerPosition.set(leftControllerPosition);
				}
			}
		}

		if (bIsRightTriggerPressed) {
			if (openVR.isControllerConnected(vr::TrackedControllerRole_RightHand)) {
				// Getting the translation component of the controller pose matrix
				rightControllerPosition = openVR.getControllerPose(vr::TrackedControllerRole_RightHand)[3];

				if (lastRightControllerPosition.distance(rightControllerPosition) >= polylineResolution) {
					rightControllerPolylines[rightControllerPolylines.size() - 1].lineTo(rightControllerPosition);
					lastRightControllerPosition = rightControllerPosition;
				}
			}
		}
		//printf("Left Controller: %f %f %f \n Right Controller: %f %f %f \n", leftControllerPosition.x, leftControllerPosition.y, leftControllerPosition.z, rightControllerPosition.x, rightControllerPosition.y, rightControllerPosition.z);
	}


	// doesn't get used at the moment
	void drawParticles() {

		int num = cDepthWidth * cDepthHeight;
		float radius = 1;
		for (int i = 0; i < 10; i++) {
			float theta1 = ofRandom(0, TWO_PI);
			float theta2 = ofRandom(0, TWO_PI);

			ofVec3f p;
			p.x = cos(theta1) * cos(theta2);
			p.y = sin(theta1);
			p.z = cos(theta1) * sin(theta2);
			p *= radius;

			addPoint(p.x, p.y, p.z);
		}
	}

	void draw() {
		ofSetupScreen();  // sets up default perspective matrix
		ofBackground(0);

		openVR.render();
		openVR.renderDistortion();

		kinectTexture[0].draw(ofGetWidth() / 2, 0, ofGetWidth()/2, ofGetHeight()/2);
		kinectTexture[1].draw(ofGetWidth() / 2, ofGetHeight() / 2, ofGetWidth() / 2, ofGetHeight() / 2);

		openVR.drawDebugInfo(10.0f, 500.0f);


		// Help
		if (bShowHelp) {
			_strHelp.str("");
			_strHelp.clear();
			_strHelp << "HELP (press h to toggle): " << endl;
			_strHelp << "Press the Trigger of a controller to draw a line with that specific controller." << endl;
			_strHelp << "Press the Touchpad to star a new line." << endl;
			_strHelp << "Press the Grip button to clear all the lines drawn with that specific controller." << endl;
			_strHelp << "Drawing resolution " << polylineResolution << " (press: +/-)." << endl;
			_strHelp << "Drawing default 3D models " << openVR.getRenderModelForTrackedDevices() << " (press: m)." << endl;
			ofDrawBitmapStringHighlight(_strHelp.str(), ofPoint(10.0f, 20.0f), ofColor(ofColor::black, 100.0f));
		}
	}

	void draw_scene(ofMatrix4x4 viewMatrix) {
		glDepthMask(GL_FALSE);
		// this makes everything look glowy :)
		ofEnableBlendMode(OF_BLENDMODE_ADD);
		ofEnablePointSprites();

		shader.begin();
		shader.setUniform1f("size", 2.f);
		shader.setUniformMatrix4f("modelViewProjectionMatrix", viewMatrix);

		glPointSize(40.f);

		texture.bind();
		vbo.draw(GL_POINTS, 0, (int)points.size());
		texture.unbind();

		shader.end();

		ofDisablePointSprites();
		ofDisableBlendMode();
		glDepthMask(GL_TRUE);

	}

	void render(vr::Hmd_Eye nEye) {
		
		ofMatrix4x4 currentViewProjectionMatrix = openVR.getCurrentViewProjectionMatrix(nEye);

		/*
		draw_scene(currentViewProjectionMatrix);

		controlShader.begin();
		//controlShader.setUniformMatrix4f("matrix", matrix);

		for (int eHand = 0; eHand <= 1; eHand++)
		{
			printf("Draw Controller %d %d %p\n", eHand, m_rHand[eHand].m_bShowController , !m_rHand[eHand].m_pRenderModel);
			
			if (!m_rHand[eHand].m_bShowController || !m_rHand[eHand].m_pRenderModel)
				continue;

			const glm::mat4 & matDeviceToTracking = m_rHand[eHand].m_rmat4Pose;
			glm::mat4 matMVP = openVR.getCurrentViewProjectionMatrix(nEye) * matDeviceToTracking;
			glUniformMatrix4fv(m_nRenderModelMatrixLocation, 1, GL_FALSE, glm::value_ptr(matMVP) );
			
			m_rHand[eHand].m_pRenderModel->Draw();
		}

		controlShader.end();

		if (0) {
			shaderP.begin();
			shaderP.setUniformMatrix4f("matrix", currentViewProjectionMatrix, 1);
			ofSetColor(ofColor::white);
			for (auto pl : leftControllerPolylines) {
				pl.draw();
			}

			for (auto pl : rightControllerPolylines) {
				pl.draw();
				//addPoint(0.1, 0.2, 0.1);
			}
			//vbo.draw(GL_POINTS, 0, (int)points.size());
			shaderP.end();
		}*/
	}

	void controllerEvent(ofxOpenVRControllerEventArgs& args) {
	//	cout << "ofApp::controllerEvent > role: " << ofToString(args.controllerRole) << " - event type: " << ofToString(args.eventType) << " - button type: " << ofToString(args.buttonType) << " - x: " << args.analogInput_xAxis << " - y: " << args.analogInput_yAxis << endl;
		
		//Left
		if (args.controllerRole == ControllerRole::Left) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsLeftTriggerPressed = true;
					//addPoint(leftControllerPosition.x * 500.f, leftControllerPosition.y * 500.f, leftControllerPosition.z * 500.f);
					//printf("Left Controller x: %f  y: %f  z: %f \n", leftControllerPosition.x, leftControllerPosition.y, leftControllerPosition.z);

					//origin = ofVec3f(leftControllerPosition.x, leftControllerPosition.y, leftControllerPosition.z);
					

					if (leftControllerPolylines.size() == 0) {
						leftControllerPolylines.push_back(ofPolyline());
						lastLeftControllerPosition.set(ofVec3f());
					}
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsLeftTriggerPressed = false;
				}
			}
			// ButtonTouchpad
			else if (args.buttonType == ButtonType::ButtonTouchpad) {
				if (args.eventType == EventType::ButtonPress) {
					leftControllerPolylines.push_back(ofPolyline());
					lastLeftControllerPosition.set(ofVec3f());
				}
			}
			// Grip
			else if (args.buttonType == ButtonType::ButtonGrip) {
				if (args.eventType == EventType::ButtonPress) {
					for (auto pl : leftControllerPolylines) {
						pl.clear();
					}

					leftControllerPolylines.clear();
				}
			}
		}

		// Right
		else if (args.controllerRole == ControllerRole::Right) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsRightTriggerPressed = true;
					//addPoint(rightControllerPosition.x * 800.f, rightControllerPosition.y * 800.f, rightControllerPosition.z * 800.f);
					//printf("Right Controller x: %f  y: %f  z: %f \n", rightControllerPosition.x, rightControllerPosition.y, rightControllerPosition.z);

					//origin = ofVec3f(rightControllerPosition.x, rightControllerPosition.y, rightControllerPosition.z);

					if (rightControllerPolylines.size() == 0) {
						rightControllerPolylines.push_back(ofPolyline());
						lastRightControllerPosition.set(ofVec3f());
					}
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsRightTriggerPressed = false;
				}
			}
			// ButtonTouchpad
			else if (args.buttonType == ButtonType::ButtonTouchpad) {
				if (args.eventType == EventType::ButtonPress) {
					rightControllerPolylines.push_back(ofPolyline());
					lastRightControllerPosition.set(ofVec3f());
				}
			}
			// Grip
			else if (args.buttonType == ButtonType::ButtonGrip) {
				if (args.eventType == EventType::ButtonPress) {
					for (auto pl : rightControllerPolylines) {
						pl.clear();
					}

					rightControllerPolylines.clear();
				}
			}
		}
	}


	CGLRenderModel *FindOrLoadRenderModel(const char *pchRenderModelName)
	{
		CGLRenderModel *pRenderModel = NULL;
		for (std::vector< CGLRenderModel * >::iterator i = m_vecRenderModels.begin(); i != m_vecRenderModels.end(); i++)
		{
			if (!stricmp((*i)->GetName().c_str(), pchRenderModelName))
			{
				pRenderModel = *i;
				break;
			}
		}

		// load the model if we didn't find one
		if (!pRenderModel)
		{
			vr::RenderModel_t *pModel;
			vr::EVRRenderModelError error;
			while (1)
			{
				error = vr::VRRenderModels()->LoadRenderModel_Async(pchRenderModelName, &pModel);
				if (error != vr::VRRenderModelError_Loading)
					break;

				ThreadSleep(1);
			}

			if (error != vr::VRRenderModelError_None)
			{
				printf("Unable to load render model %s - %s\n", pchRenderModelName, vr::VRRenderModels()->GetRenderModelErrorNameFromEnum(error));
				return NULL; // move on to the next tracked device
			}

			vr::RenderModel_TextureMap_t *pTexture;
			while (1)
			{
				error = vr::VRRenderModels()->LoadTexture_Async(pModel->diffuseTextureId, &pTexture);
				if (error != vr::VRRenderModelError_Loading)
					break;

				ThreadSleep(1);
			}

			if (error != vr::VRRenderModelError_None)
			{
				printf("Unable to load render texture id:%d for render model %s\n", pModel->diffuseTextureId, pchRenderModelName);
				vr::VRRenderModels()->FreeRenderModel(pModel);
				return NULL; // move on to the next tracked device
			}

			pRenderModel = new CGLRenderModel(pchRenderModelName);
			if (!pRenderModel->BInit(*pModel, *pTexture))
			{
				printf("Unable to create GL model from render model %s\n", pchRenderModelName);
				delete pRenderModel;
				pRenderModel = NULL;
			}
			else
			{
				m_vecRenderModels.push_back(pRenderModel);
			}
			vr::VRRenderModels()->FreeRenderModel(pModel);
			vr::VRRenderModels()->FreeTexture(pTexture);
		}
		return pRenderModel;
	}

	//--------------------------------------------------------------
	

	void toggleFullScreen() { fullScreen(!isFullscreen); }

	void fullScreen(bool fs=true) {
		if (fs == isFullscreen) return;
		isFullscreen = fs;
		if(!isFullscreen) {
			ofSetWindowShape(300,300);
			ofSetFullscreen(false);
			//ofSetWindowPosition(100, 100);
		} else {
			ofSetFullscreen(true);
		}
	}

	void keyPressed(int key) {
		switch (key) {
		case 'f':
			toggleFullScreen();
			break;
		case '+':
		case '=':
			polylineResolution += .0001f;
			break;

		case '-':
		case '_':
			polylineResolution -= .0001f;
			if (polylineResolution < 0) {
				polylineResolution = 0;
			}
			break;

		case 'h':
			bShowHelp = !bShowHelp;
			break;

		case 'm':
			openVR.setRenderModelForTrackedDevices(!openVR.getRenderModelForTrackedDevices());
			break;
		default:
			break;
		}
		

	}
	// void keyReleased(int key);
	// void mouseMoved(int x, int y );
	// void mouseDragged(int x, int y, int button);
	// void mousePressed(int x, int y, int button);
	// void mouseReleased(int x, int y, int button);
	// void mouseEntered(int x, int y);
	// void mouseExited(int x, int y);
	// void windowResized(int w, int h);
	// void dragEvent(ofDragInfo dragInfo);
	// void gotMessage(ofMessage msg);
	

};
