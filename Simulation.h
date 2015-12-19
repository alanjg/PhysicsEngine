#ifndef SIMULATION_H
#define SIMULATION_H

#include <vector>

#include "RigidBody.h"
#include "Solver.h"

namespace physics
{

	class Simulation
	{
		friend class Solver;
	public:
		Simulation();
		~Simulation();
		void AddObject(RigidBody* object);
		void Update(float time,int depth=0);
		//float BinarySearch(float begin,float end,RigidBody* a,RigidBody* b);
		void Render();
	private:
		std::vector<RigidBody*> objects_;
		std::vector<RigidBody*> nextObjects_;
		Solver *solver_;
	};

} // namespace physics

#endif