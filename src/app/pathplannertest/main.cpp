/**
 *
 *
 * \date 02/2015
 * \author CEA/DRT/LIST/DIASI/LRI
 * \author E.Lucet
 *
 * @par Licence
 * Copyright © 2015 CEA
 */
//==============================================================================
// MOBILE ROBOT PATH FOLLOWING APPLICATION
//==============================================================================

#include "sys/timer.h"

#include "RobotCreator.hpp"

int main()
{

	/*
	Pour tester les collisions, on pilote le vehicule a la spacemouse.
	*/
	/*
	std::vector<xde::hardware::SpaceMouseDesc> smds = xde::hardware::SpaceMouse::scan();
	xde::hardware::SpaceMouse sm(smds[0]);
	sm.setMaxLinearVelocity(1.);
	sm.setMaxAngularVelocity(3.14);
	sm.setDominatingModeOn();

	xde::manip::VelocityBasedManipulator vbm(mechanicalScene);
	vbm.setTau(.01);
	*/
	/*vbm.attach(vehicleBody);
	vehicleBody.disableWeight();*/

	//std::cout << "Building spacemouse: " << std::endl;
	


	/*
	Boucle de simulation.
	*/

	try
	{
		RobotCreator vehicle;

		for(int i = 0; i < 5000; ++i)
		{
			vehicle.update();
		}

		vehicle.finalize();
		system("PAUSE");
	}
	catch (std::exception & e)
    {
		std::cerr << "Got exception: " << e.what() << std::endl;
    }

	return 0;
}
