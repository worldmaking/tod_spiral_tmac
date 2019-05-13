#pragma once

#include "ofMain.h"
#include "ofxOpenVR.h"
#include "of3dGraphics.h"

#include "al/al_kinect2.h"

#include "shared.h"
#include "simulation.h"

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/core/affine.hpp"

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

/*
	al::Isosurface iso;
	iso.level(isolevel=0.);
	// field is a DIM*DIM*DIM array of single floats
	// voxelsize is size of a voxel in opengl units
	generate(field, DIM, voxelsize=1.);

*/
class IsosurfaceVbo {
public:
	
	//gl::VboMeshRef mVboMesh;
	ofVbo vbo;
	
	void update_gpu(al::Isosurface &iso) {
		vbo.setNormalData(&iso.vertices().elems()->normal.x, iso.vertices().size(), GL_DYNAMIC_DRAW, sizeof(al::Isosurface::VertexData));
		vbo.setVertexData(&iso.vertices().elems()->position.x, 3, iso.vertices().size(), GL_DYNAMIC_DRAW, sizeof(al::Isosurface::VertexData));
		vbo.setIndexData(iso.indices().elems(), iso.indices().size(), GL_DYNAMIC_DRAW);
	}
	
	void draw() {
		vbo.drawElements(GL_TRIANGLES, vbo.getNumIndices());
	}
};


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

struct calibrationPoints
{
	std::vector<glm::vec3> vrPositions;
	std::vector<glm::vec3> kinectPositions;
};

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
	bool bIsLeftTouchpadPressed = false;
	bool bIsRightTouchpadPressed = false;
	bool waitForPadL = false;
	bool waitForPadR = false;
	bool bIsLeftGripPressed = false;
	bool bIsRightGripPressed = false;
	bool waitForGripL = false;
	bool waitForGripR = false;
	ofVec3f leftControllerPosition;
	ofVec3f rightControllerPosition;
	ofVec3f lastLeftControllerPosition;
	ofVec3f lastRightControllerPosition;
	std::vector< CGLRenderModel * > m_vecRenderModels;
	//CGLRenderModel *FindOrLoadRenderModel(const char *pchRenderModelName);

	bool calibrate = true;
	bool calZero = false;
	bool calOne = false;


	calibrationPoints kinectCalibrator[2];

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

	bool isCapturing;

	bool rightDown = false;
	bool leftDown = false;

	glm::mat4x4 controllerBegin;
	glm::mat4x4 kinectBegin;

	ofVec3f origin = ofVec3f(-1.f, 0.f, -1.f); //starts the kinect space closer to the kinect (The physical kniect is somewhere < -1,0,-1 in the VR space)

	glm::vec3 kinectPosition[2];
	glm::vec3 controllerDragStartPosition;
	
	IsosurfaceVbo vboIso;
	ofShader shaderIso;

	glm::mat4 transformedPoints;
	glm::mat4 viewMatrix;

	void setup() {
		Simulation& sim = Simulation::get();
		sim.setup();

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
		for (int i = 0; i < cloudDeviceManager.numDevices; i++) {
			CloudDevice& kinect = cloudDeviceManager.devices[i];
			kinect.use_colour = false;
		}
		cloudDeviceManager.open_all();

		// upload the data to the vbo
		int total = (int)points.size();
		vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
		vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);
		shader.load("shaders_gl3/point");

		// upload isosurface data to gpu
		vboIso.update_gpu(sim.isosurface);
		shaderIso.load("shaders_gl3/iso");

		//double ransacThreshold = 3;
		//double confidence = 0.99;
		//std::vector<cv::Point3f> src, dst;
		//src.push_back(cv::Point3f(1, 0, 0));
		//src.push_back(cv::Point3f(0, 1, 0));
		//src.push_back(cv::Point3f(0, 0, 1));
		//src.push_back(cv::Point3f(1, 0, 1));
		////src.push_back(cv::Point3f(1, 1, 1));

		//dst.push_back(cv::Point3f(1, 0, 0));
		//dst.push_back(cv::Point3f(0, 1, 0));
		//dst.push_back(cv::Point3f(0, 0, 1));
		//dst.push_back(cv::Point3f(1, 0, 1));
		////dst.push_back(cv::Point3f(1, 1, 1));

		//cv::Mat aff(3, 4, CV_64F);
		//std::vector<uchar> inliers;
		//// res should be 1 for OK
		//// inliers.size() tells us how many of the given points were successfully matched
		//int res = cv::estimateAffine3D(src, dst, aff, inliers, ransacThreshold, confidence);
		////printf("result %d %d\n", res, inliers.size());
		////std::cout << aff << std::endl;
		
	}

	void addPoint(float x, float y, float z) {
		ofVec3f p(x, y, z);
		points.push_back(p);

		// we are passing the size in as a normal x position
		float size = 1;
		sizes.push_back(ofVec3f(size));
	}
	
	void calculateTranslations() {
		double ransacThreshold = 3;
		double confidence = 0.99;
		std::vector<cv::Point3f> src, dst;
		src.push_back(cv::Point3f(1, 0, 0));
		src.push_back(cv::Point3f(0, 1, 0));
		src.push_back(cv::Point3f(0, 0, 1));
		src.push_back(cv::Point3f(1, 0, 1));
		//src.push_back(cv::Point3f(1, 1, 1));

		dst.push_back(cv::Point3f(1, 0, 0));
		dst.push_back(cv::Point3f(0, 1, 0));
		dst.push_back(cv::Point3f(0, 0, 1));
		dst.push_back(cv::Point3f(1, 0, 1));
		//dst.push_back(cv::Point3f(1, 1, 1));

		cv::Mat aff(3, 4, CV_64F);
		std::vector<uchar> inliers;
		// res should be 1 for OK
		// inliers.size() tells us how many of the given points were successfully matched
		int res = cv::estimateAffine3D(src, dst, aff, inliers, ransacThreshold, confidence);
		printf("result %d %d\n", res, inliers.size());
		std::cout << aff << std::endl;
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

		vector <ofVec3f> pointsUpdate;
		vector <ofVec3f> sizesUpdate;
		
		for (int i = 0; i < 2; i++) {
			CloudDevice& kinect = cloudDeviceManager.devices[i];
			if (kinect.capturing) {

				isCapturing = true; 
				// get most recent frames:
				const CloudFrame& cloud = kinect.cloudFrame();
				const ColourFrame& colour = kinect.colourFrame();

				int num = cDepthWidth * cDepthHeight;
				float radius = 1;

				//glm::mat4 cloudTransform = glm::mat4(1.); this is what kinect2 auto sets the cloud transform to be

				//Calibration checker
				if ((calZero && i == 0) || (calOne && i == 1)) {
					if (bIsLeftTriggerPressed && calibrate) {

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

							// update our cloud [Moves kinect #1]:
							kinectPosition[i] += delta;
						}

						kinect.cloudTransform = glm::translate(glm::mat4(1.), kinectPosition[i]);

						//printf("moved to %s\n", glm::to_string(kinect.cloudTransform).data());

					}
					else leftDown = false;

					if (bIsRightTriggerPressed && calibrate) {

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

							// update our cloud [Moves kinect #2]:
							kinectPosition[i] += delta * 3;
						}

						kinect.cloudTransform = glm::translate(glm::mat4(1.), kinectPosition[i]);

						//printf("moved to %s\n", glm::to_string(kinect.cloudTransform).data());
					}
					else rightDown = false;

					//grabbing left or right controller position and kinect position with touchpad
					if (bIsLeftTouchpadPressed && calibrate && !waitForPadL) {
						kinectCalibrator[i].vrPositions.push_back(glm::vec3(openVR.getControllerPose(vr::TrackedControllerRole_LeftHand)[3]));
						kinectCalibrator[i].kinectPositions.push_back(kinectPosition[i]);
						printf("Added VR position: %f, %f, %f for kinect %d\n", kinectCalibrator[i].vrPositions.back().x, kinectCalibrator[i].vrPositions.back().y, kinectCalibrator[i].vrPositions.back().z, i);
						printf("Added Kinect position: %f, %f, %f for kinect %d\n", kinectCalibrator[i].kinectPositions.back().x, kinectCalibrator[i].kinectPositions.back().y, kinectCalibrator[i].kinectPositions.back().z, i);
						waitForPadL = true;
					}
					else if (!bIsLeftTouchpadPressed)
						waitForPadL = false;
					if (bIsRightTouchpadPressed && calibrate && !waitForPadR) {
						kinectCalibrator[i].vrPositions.push_back(glm::vec3(openVR.getControllerPose(vr::TrackedControllerRole_RightHand)[3]));
						kinectCalibrator[i].kinectPositions.push_back(kinectPosition[i]);
						printf("Added VR position: %f, %f, %f for kinect %d\n", kinectCalibrator[i].vrPositions.back().x, kinectCalibrator[i].vrPositions.back().y, kinectCalibrator[i].vrPositions.back().z, i);
						printf("Added Kinect position: %f, %f, %f for kinect %d\n", kinectCalibrator[i].kinectPositions.back().x, kinectCalibrator[i].kinectPositions.back().y, kinectCalibrator[i].kinectPositions.back().z, i);
						waitForPadR = true;
					}
					else if (!bIsRightTouchpadPressed)
						waitForPadR = false;

					if (bIsLeftGripPressed && calibrate && !waitForGripL && kinectCalibrator[i].vrPositions.size() > 0) {
						kinectCalibrator[i].vrPositions.pop_back();
						kinectCalibrator[i].kinectPositions.pop_back();
						waitForGripL = true;
					}
					else if (!bIsLeftGripPressed)
						waitForGripL = false;
					if (bIsRightGripPressed && calibrate && !waitForGripR && kinectCalibrator[i].vrPositions.size() > 0) {
						kinectCalibrator[i].vrPositions.pop_back();
						kinectCalibrator[i].kinectPositions.pop_back();
						waitForGripR = true;
					}
					else if (!bIsRightGripPressed)
						waitForGripR = false;
				}

				if (kinectCalibrator[i].kinectPositions.size() >= 4) {

					calculateTranslations();
					int size = kinectCalibrator[i].vrPositions.size();
					for (int i = size-1; i > 0; i--) {
						kinectCalibrator[i].vrPositions.pop_back();
						kinectCalibrator[i].kinectPositions.pop_back();
					}


				}

				for (int j = 0; j < num; j++) {
					ofVec3f p;
					p.x = cloud.xyz[j].x + origin.x;
					p.y = cloud.xyz[j].y + origin.y;
					p.z = cloud.xyz[j].z + origin.z;

					pointsUpdate.push_back(p);
					float size = 1;
					sizesUpdate.push_back(ofVec3f(i, !i, size));
				}

				for (int j = 0; j < kinectCalibrator[i].vrPositions.size(); j++) {
					ofVec3f p;
					p.x = kinectCalibrator[i].vrPositions.at(j).x;
					p.y = kinectCalibrator[i].vrPositions.at(j).y;
					p.z = kinectCalibrator[i].vrPositions.at(j).z;
					float size = 10;
					sizesUpdate.push_back(ofVec3f(i, !i, size));
					
					pointsUpdate.push_back(p);
				}
			}
		}

		updatePoints(pointsUpdate, sizesUpdate);
		int total = (int)points.size();
		vbo.setVertexData(&points[0], total, GL_STATIC_DRAW);
		vbo.setNormalData(&sizes[0], total, GL_STATIC_DRAW);

		if (isFullscreen){
			ofHideCursor();
		} else {
		 	ofShowCursor();
		}
		openVR.update();

		/*for (int eHand = 0; eHand <= 1; eHand++)
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
		}*/

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

	void draw_scene(glm::mat4 viewMatrix, glm::mat4 projMatrix) {
		// TODO: is this in Simulation/shared?
		double now_s = ofGetElapsedTimeMillis() * 0.001;
		glm::mat4 viewProjectionMatrix = projMatrix * viewMatrix;

		

		glDepthMask(GL_FALSE);
		// this makes everything look glowy :)
		ofEnableBlendMode(OF_BLENDMODE_ADD);

		shaderIso.begin();
		shaderIso.setUniformMatrix4f("ciViewMatrix", viewMatrix);
		shaderIso.setUniformMatrix4f("ciProjectionMatrix", projMatrix);
		shaderIso.setUniform1f("uNow", now_s);
		// TODO:
		//shaderIso.setUniform1i("uGradient", 0);
		// bind mGooTex
		{ // wireframe
			shaderIso.setUniform1f("uAlpha", 0.15f);
			glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
			vboIso.draw();
		}
		shaderIso.setUniform1f("uAlpha", 0.2f);
		glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
		vboIso.draw();
		shaderIso.end();

		ofEnablePointSprites();

		shader.begin();
		shader.setUniform1f("size", 2.f);
		shader.setUniformMatrix4f("modelViewProjectionMatrix", viewProjectionMatrix);

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
		
		viewMatrix = openVR.getCurrentViewMatrix(nEye);
		glm::mat4 projMatrix = openVR.getCurrentProjectionMatrix(nEye);

		
		draw_scene(viewMatrix, projMatrix);

		/*controlShader.begin();
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

		controlShader.end();*/
	}

	void controllerEvent(ofxOpenVRControllerEventArgs& args) {
	//	cout << "ofApp::controllerEvent > role: " << ofToString(args.controllerRole) << " - event type: " << ofToString(args.eventType) << " - button type: " << ofToString(args.buttonType) << " - x: " << args.analogInput_xAxis << " - y: " << args.analogInput_yAxis << endl;
		
		//Left
		if (args.controllerRole == ControllerRole::Left) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsLeftTriggerPressed = true;

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
					bIsLeftTouchpadPressed = true;
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsLeftTouchpadPressed = false;
				}
			}
			// Grip
			else if (args.buttonType == ButtonType::ButtonGrip) {
				if (args.eventType == EventType::ButtonPress) {
					for (auto pl : leftControllerPolylines) {
						pl.clear();
					}
					leftControllerPolylines.clear();

					bIsLeftGripPressed = true;
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsLeftGripPressed = false;
				}
			}
		}

		// Right
		else if (args.controllerRole == ControllerRole::Right) {
			// Trigger
			if (args.buttonType == ButtonType::ButtonTrigger) {
				if (args.eventType == EventType::ButtonPress) {
					bIsRightTriggerPressed = true;

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
					bIsRightTouchpadPressed = true;
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsRightTouchpadPressed = false;
				}
			}
			// Grip
			else if (args.buttonType == ButtonType::ButtonGrip) {
				if (args.eventType == EventType::ButtonPress) {
					for (auto pl : rightControllerPolylines) {
						pl.clear();
					}
					rightControllerPolylines.clear();

					bIsRightGripPressed = true;
				}
				else if (args.eventType == EventType::ButtonUnpress) {
					bIsRightGripPressed = false;
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

		case 'c':
			calibrate = !calibrate;
			kinectCalibrator[0].kinectPositions.clear();
			kinectCalibrator[0].vrPositions.clear();
			kinectCalibrator[1].kinectPositions.clear();
			kinectCalibrator[1].vrPositions.clear();
			printf("Calibration is %s\n", calibrate ? "true" : "false");
			break;

		case '0':
			calZero = true;
			calOne = false;
			printf("Calibration of Kinect 0\n");
			break;
		case '1':
			calZero = false;
			calOne = true;
			printf("Calibration of Kinect 1\n");
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
