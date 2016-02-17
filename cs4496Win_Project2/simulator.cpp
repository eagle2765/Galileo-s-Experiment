#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(4);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = -0.5;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = -0.2;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.1;
    mParticles[2].mPosition[1] = 20.0;
	mParticles[3].mPosition[0] = 0.4;
	mParticles[3].mPosition[1] = 20.0;
    
    // Init particle colors (default is red)
    mParticles[1].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    mParticles[2].mColor = Eigen::Vector4d(0.2, 0.2, 0.9, 1.0); // Blue
    
    mTimeStep = 0.03;
    mElapsedTime = 0;
}

int Simulator::getNumParticles() {
    return mParticles.size();
}

Particle* Simulator::getParticle(int index) {
    return &mParticles[index];
}

double Simulator::getTimeStep() {
    return mTimeStep;
}

void Simulator::reset() {
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = 20.0;
    
    for (int i = 0; i < 3; i++) {
        mParticles[i].mVelocity.setZero();
        mParticles[i].mAccumulatedForce.setZero();
    }
    
    mElapsedTime = 0;
}

void Simulator::simulate() {

	/* Particle 1 (Red) : Theoretical
	   Particle 2 (Blue) : Midpoint
	   Particle 3 (Blue) : Euler's Explicit
	   Particle 4 (Red ): Euler's Implicit

	   Both Euler's should be more inaccurate as the timestep size increases. Euler's explicit will be slow because it is using the initial
	   velocity in the time slice for approximation. Euler's implicit should be fast because it is using the final velocity in the time slice for approximation. 
	*/

	// Theoretical acceleration of 9.8 m/s
	double v0 = mParticles[0].mVelocity[1];
	double x0 = mParticles[0].mPosition[1];

	mParticles[0].mVelocity[1] = v0 + (-1 * 9.8) * mTimeStep;
	mParticles[0].mPosition[1] = x0 + v0 * mTimeStep + 0.5 * (- 1 * 9.8) * (mTimeStep * mTimeStep);

	// Midpoint method of integration
	v0 = mParticles[1].mVelocity[1];
	x0 = mParticles[1].mPosition[1];

	// Calculate velocity final
	mParticles[1].mVelocity[1] = v0 + (-1 * 9.8) * mTimeStep;
	double vf = mParticles[1].mVelocity[1];

	//midpoint formula
	mParticles[1].mPosition[1] = x0 + 0.5 * (v0 + vf) * mTimeStep;


	// Explicit Euler method
	v0 = mParticles[2].mVelocity[1];
	x0 = mParticles[2].mPosition[1];

	mParticles[2].mPosition[1] = x0 + mTimeStep * v0;
	mParticles[2].mVelocity[1] = v0 + mTimeStep * (-1 * 9.8);
	

	// Implicit Euler method
	v0 = mParticles[3].mVelocity[1];
	x0 = mParticles[3].mPosition[1];

	mParticles[3].mVelocity[1] = v0 + mTimeStep * (-1 * 9.8);
	mParticles[3].mPosition[1] = x0 + mTimeStep * mParticles[3].mVelocity[1];
	
	
    
    mElapsedTime += mTimeStep;
}







