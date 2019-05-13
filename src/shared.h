#ifndef __tod_shared__
#define __tod_shared__

#include "al/al_math.h"
#include "al/al_field3d.h"

//#define AN_USE_CV_FLOW

#include <deque>

// Field ffi:
typedef Fluid3D<float> al_Fluid;
typedef Field3D<float> al_Field;

static const int RSHIFT = 5;
static const int RSHIFT2 = 10;
static const float NEARCLIP = 0.1;
static const float FARCLIP = 100;

static const int DIM = 32;
static const int DIMWRAP = 31;
static const int DIM2 = 1024;
static const int DIM3 = 32768;

static const int DIMHALF = 16;
static const int DIMHALF2 = 256;
static const int DIMHALF3 = 4096;

static const int NUM_VOXELS = 32 * 32 * 32;
static const int VOXEL_MAX = 7;
static const uint32_t INVALID_VOXEL_HASH = 0xffffffff;

static const int NUM_PARTICLES = 20000;
static const int MAX_KINECT_POINTS = 307200;

static const int NUM_BEETLES = 2048;
static const int NUM_SNAKES = 7;
static const int SNAKE_MAX_SEGMENTS = 17;
static const int NUM_SNAKE_SEGMENTS = 136;
static const int MAX_GRAINS = 256;

static const int NUM_GHOSTPOINTS = 320000;

#define OSC_TABLE_DIM 2048
#define OSC_TABLE_WRAP (OSC_TABLE_DIM-1)

struct Ghost {
	glm::vec4 position;
};

struct Particle {
	glm::vec4 pos;
	
	//glm::vec4 color;
	
	float energy, age, agelimit, dead;
	
	glm::vec3 dpos;
	
	uint32_t cloud;
	
	struct Particle * next;
};

struct BeetleInstanceData {
	glm::vec4 vInstanceOrientation;
	glm::vec4 vInstanceColor;
	glm::vec3 vInstancePosition;
	float vInstanceAge;
	glm::vec3 vInstanceScale;
	float pad;
};

typedef struct Beetle {
	
	glm::quat orientation;
	glm::vec4 color;
	glm::vec3 pos;	float pospad;
	glm::vec3 scale; float scalepad;
	
	glm::vec3 vel; float velpad;
	glm::vec3 angvel; float flowsensor;
	
	//float noise, snoise;
	float accelerometer;
	float energy, wellbeing, wellbeingprev;
	float age;
	
	float acceleration, azimuth, elevation, bank;
	//float memory[16];
	
	int32_t id, at, alive, recycle;
	
	struct Particle * nearest_particle;
	
	struct Beetle * next; // for voxels
	
	// audio grain properties:
	float period;
	
	glm::vec4 grain_mix;
	int32_t grain_start, grain_remain, grain_active;
	float grain_modulate;
	
	float grain_oinc, grain_ophase, grain_einc, grain_ephase;
	
	float grain_filter, grain_smoothed;

} Beetle;

typedef struct SnakeSegment {
	glm::quat orientation;
	glm::vec4 color;
	glm::vec3 pos; float phase;
	glm::vec3 scale;
} SnakeSegment;

typedef struct Snake {
	glm::vec3 wtwist; float length;
	float pincr, energy;
	
	SnakeSegment * segments[SNAKE_MAX_SEGMENTS];
	Beetle * victim;
	Beetle * nearest_beetle;
	
	int32_t id;
} Snake;

typedef struct Voxel {
	Particle * particles;
	Beetle * beetles;
} Voxel;

typedef struct KinectData {
	
	// world-location corresponding to depth pixel:
	glm::vec3 world[640*480];
	
	// smoothing the depth stream:
	uint16_t prev[640*480], filt[640*480];
	
	// for CV processing
	uint8_t hscur[320*240];
	uint8_t hsprev[320*240];
	float hsvx[320*240];
	float hsvy[320*240];

	glm::vec3 location, euler, scale;
	float pad;

} KinectData;

typedef struct Shared {
	Voxel voxels[NUM_VOXELS];
	float human_present[NUM_VOXELS];
	Particle particles[NUM_PARTICLES];
	Beetle beetles[NUM_BEETLES];
	BeetleInstanceData beetleInstances[NUM_BEETLES];
	
	SnakeSegment snakesegments[NUM_SNAKE_SEGMENTS];
	Snake snakes[NUM_SNAKES];
	Ghost ghostpoints[NUM_GHOSTPOINTS];
	
	Fluid3D<float> fluid;
	Field3D<float> landscape;
	Array<float> noisefield;
	
	// for the ghost clouds, as voxel fields
	// (double-buffered)
	Field3D<float> density;
	Field3D<glm::vec3> density_gradient;
	Array<glm::vec3> density_change;
	
	float mEnvTable[OSC_TABLE_DIM];
	float mOscTable[OSC_TABLE_DIM];
	
	KinectData kdata[2];
	
	uint32_t beetleList, wingList, snakeList, tetraList, screenList;
	
	double fps, now;
	float goo_rate;
	
	float particle_agelimit;
	float particle_entropy, particle_life_threshold, particle_noise, particle_move_scale;
	float beetle_size, beetle_decay, beetle_push, beetle_friction, beetle_friction_turn, beetle_reproduction_threshold, beetle_speed;
	float beetle_life_threshold, beetle_max_acceleration, beetle_max_turn;
	float beetle_base_period, beetle_modulation;
	float beetle_dur, beetle_frequency, beetle_rpts, beetle_filter;
	float snake_decay, snake_hungry;
	float fluid_amount, fluid_viscocity, fluid_advection, fluid_decay;
	float fluid_boundary_friction;
	float snake_fluid_suck;
	float human_smoothing, human_flow, human_cv_flow, human_fluid_smoothing;
	
	float density_isolevel, density_ideal, density_diffuse, density_decay;
	
	uint32_t live_beetles, gooRequested, gooReady, ghostcount;
	
	double samplerate, audio_gain, motor_angle;
	
	double parallax_range, parallax_rate;
	float view_zoom, view_distance, view_eye_distance;
	float eye_height;
	
	glm::mat4 projection0, projection1, view0, view1;
	glm::mat4 projection0_walls, projection1_walls;
	
	glm::vec3 world_dim, center, snake_levity;
	
	uint32_t fluid_passes;
	
	std::deque<int32_t> beetle_pool;
	
	size_t blocksize;
	

	float strafey;
	
	bool updating;
	
	Shared();
	
	void update_fluid(double dt);
	void update_density(double dt);
	
	void update_goo(double dt);
	void update_particles(double dt);
	
	void update_beetles(double dt);
	void beetle_birth(Beetle& self);
	
	void update_snakes(double dt);
	
	void update_move(double dt);
	void move_particles(double dt);
	void move_beetles(double dt);
	void move_snakes(double dt);
	
	
	void dsp_initialize(double sr, long blocksize);
	void perform_audio(float * FL, float * FR, float * BL, float * BR, long frames);
	
	inline void beetle_pool_push(Beetle& b) {
		//printf("push beetle %d %d\n", b, app->beetle_pool.size());
		beetle_pool.push_back(b.id);
		b.recycle = 1;
		b.alive = 0;
	}
	
	void beetle_pool_clear() {
		while (!beetle_pool.empty()) beetle_pool_pop();
	}
	
	inline int32_t beetle_pool_pop() {
		int b = -1;
		if (!beetle_pool.empty()) {
			b = beetle_pool.front();
			beetle_pool.pop_front();
			beetles[b].recycle = 0;
		}
		return b;
	}
	
	
	inline uint32_t voxel_hash(glm::vec3 pos) { return voxel_hash(pos.x, pos.y, pos.z); }
	
	inline uint32_t voxel_hash(float px, float py, float pz) {
		static const int32_t DIMY = DIM;
		static const int32_t DIMZ = DIM;
		static const int32_t DIMX = DIM;
		
		int32_t x = px, y = py, z = pz;
		if (x < 0 || x >= DIMX || y < 0 || y >= DIMY || z < 0 || z >= DIMZ) {
			return INVALID_VOXEL_HASH;
		}
		return x + DIMX*(y + (DIMY*z));
	}
	
	inline void voxel_push(Voxel& v, Particle& p) {
		p.next = v.particles;
		v.particles = &p;
	}
	
	inline void voxel_push(Voxel& v, Beetle& p) {
		p.next = v.beetles;
		v.beetles = &p;
	}
	
	inline Particle * voxel_pop_particle(Voxel& v) {
		Particle * p = v.particles; // the top particle
		if (p) {
			v.particles = p->next;
			p->next = 0;
		}
		return p;
	}
	
	inline Beetle * voxel_pop_beetle(Voxel& v) {
		Beetle * p = v.beetles; // the top particle
		if (p) {
			v.beetles = p->next;
			p->next = 0;
		}
		return p;
	}
	
	inline void clear_voxels() {
		memset(voxels, 0, sizeof(voxels));
	}
	
	
	glm::vec3 fluid_velocity(const glm::vec3& pos) {
		glm::vec3 flow;
		fluid.velocities.front().read_interp<float>(pos.x, pos.y, pos.z, &flow.x);
		return flow;
	}
	
	void apply_fluid_boundary(glm::vec3 * velocities, const float * landscape, const size_t dim0, const size_t dim1, const size_t dim2);
	
	
} Shared;


#endif
