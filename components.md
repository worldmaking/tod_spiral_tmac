
Devices:

- Input: 
	- 2x or 3x kinects, calibrated, clipped/filtered, merged into single point cloud
	- Vive tracking, also calibrated in consistent space (i.e. same on each run)

- Output:
	- 2x projectors (i.e. two windows, full-screened, on separate displays)
	- 1x Vive HMD plus controllers, distinct render to the two displays
	- Multi-channel audio (probably 4.1?)

Rendering/shading:
- Grids for rooms
- Point sprite for ghost-cloud
- Point sprites for particles
- Mesh/DF for beetles
- Mesh/DF for snakes
- Isosurface/DF for slime
- versions of above for shadowFBO
- maybe deferred render system for better lighting?


App modes:
- Calibration mode
- Kinect/Vive capture / playback mode(s)
- Debug display mode(s)
- Single display/no VR modes?
- Final work



