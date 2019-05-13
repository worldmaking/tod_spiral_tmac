#ifndef TOD_SIMULATION_H
#define TOD_SIMULATION_H

#include "ofMain.h"
#include "ofxOpenVR.h"
#include "of3dGraphics.h"

#include "shared.h"
#include "al/al_isosurface.h"

struct Simulation {

	std::thread mSimulationThread, mFluidThread, mHumanThread;
	float mSimulationSeconds, mFluidSeconds, mHumanSeconds;

    al::Isosurface isosurface;

    static Simulation& get();

    void setup() {

        // initialize isosurface
        {
            isosurface.vertices().resize(5 * DIM*DIM*DIM);
            isosurface.indices().resize(3 * isosurface.vertices().size());
            
            // generate some demo data
            std::vector<float> volData;
            volData.resize(DIM*DIM*DIM);
            static const int N = DIM;
            static float phase = 0;
            if((phase += 0.0002) > 2*M_PI) phase -= 2*M_PI;
            for(int k=0; k<N; ++k){ double z = double(k)/N * 4*M_PI;
                for(int j=0; j<N; ++j){ double y = double(j)/N * 4*M_PI;
                    for(int i=0; i<N; ++i){ double x = double(i)/N * 4*M_PI;                   
                        volData[k*N*N + j*N + i]
                        = cos(x * cos(phase*7))
                        + cos(y * cos(phase*8))
                        + cos(z * cos(phase*9));
                    }
                }
            }
			isosurface.level(0.);
			isosurface.generate(&volData[0], DIM, 1.);
        }
    }
};

#endif