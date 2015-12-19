#ifndef SOLVER_H
#define SOLVER_H

#include "Simulation.h"

namespace physics
{
	class Simulation;

	class Solver
	{
	public:
		Solver(Simulation* simulation)
		{
			simulation_ = simulation;
		}
		void Update(float time);
        void LockInUpdate();
	private:
		Simulation* simulation_;
	};
} // namespace physics

#endif