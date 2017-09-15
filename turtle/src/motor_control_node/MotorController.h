#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H


#include "CML.h"
#include "can_kvaser.h"
CML_NAMESPACE_USE();

/// Class for mecanum-wheel control
class MotorController {
public:
	/// Constructor
	MotorController();

	/// Destructor
	~MotorController();

	/// Initial
    void initialize(int canBPS) ;

	/// If Succeed
	bool isSucceed();

	/// Set Velocity
	void setLaserMotorVelocity(double vel);

	/// Get Velocity
	void getLaserMotorVelocity(double& vel);

	/// Get Position
	void getLaserMotorPosition(double& pos);

    void doMove(ProfileConfigTrap &cfg);

private:
	/// Amplifier group
	Amp			laserMotorAmp;
    /// can network
    CanOpen net;
    /// can interface
    KvaserCAN hw;
	/// Succeed
	bool		m_succeed;
};



#endif	
