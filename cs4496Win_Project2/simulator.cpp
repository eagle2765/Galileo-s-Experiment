#include "simulator.h"
#include <fstream>
#include <iostream>

using namespace std;

Simulator::Simulator() {
    // initialize the particles
    mParticles.resize(3);
    
    // Init particle positions (default is 0, 0, 0)
    mParticles[0].mPosition[0] = -0.3;
    mParticles[0].mPosition[1] = 20.0;
    mParticles[1].mPosition[0] = 0.0;
    mParticles[1].mPosition[1] = 20.0;
    mParticles[2].mPosition[0] = 0.3;
    mParticles[2].mPosition[1] = 20.0;
    
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
    // TODO: Replace the following code
	/*
    for (int i = 0; i < mParticles.size(); i++) {
        mParticles[i].mPosition[1] -= 0.005;
    }
	*/
	//mParticles[0].mPosition[1] -= 0.007;
	//mParticles[1].mPosition[1] -= 0.010;
	//mParticles[2].mPosition[1] -= 0.005;

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
	/*
	v0 = mParticles[2].mVelocity[1];
	x0 = mParticles[2].mPosition[1];

	mParticles[2].mPosition[1] = x0 + mTimeStep * v0;
	mParticles[2].mVelocity[1] = v0 + mTimeStep * (-1 * 9.8);
	*/
    
    mElapsedTime += mTimeStep;
}







