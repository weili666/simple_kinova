#include "MotorController.h"
#include <stdio.h>

/// Constructor
MotorController::MotorController() {
	m_succeed = false;
}

/// Destructor
MotorController::~MotorController() {

}

/// Initial
void MotorController::initialize(int canBPS) {
	/// Reset flag
    m_succeed = false;

    hw.SetName("CAN0");
    hw.SetBaud( canBPS );

    const Error *err = net.Open( hw );
    if(err )printf( "Can Open Error: %s\n", err->toString() );

    int laserMotorID = 1;  //can Node ID: laser motor:1

    //初始化laser motor
    err=laserMotorAmp.Init(net,laserMotorID);
    if (err) printf("laserMotorAmp - Error : %s\n", err->toString());

    //  Home the motor.
     HomeConfig hcfg;

     err = laserMotorAmp.GoHome( hcfg );
      if(err )printf( "Go home  Error: %s\n", err->toString() );

     printf( "Waiting for home to finish...\n" );
     err = laserMotorAmp.WaitMoveDone( 20000 );
     if(err )printf( "Go home  Error: %s\n", err->toString() );


    err = laserMotorAmp.Enable();
    err = laserMotorAmp.SetAmpMode(AMPMODE_CAN_VELOCITY);
    err = laserMotorAmp.SetHaltMode(HALT_DISABLE);

    if(err)printf( "set mode  Error: %s\n", err->toString() );


    printf("MotorController has been initialized ...\n");

    if(!err)m_succeed = true;

	return;
}

/// If Succeed
bool MotorController::isSucceed()
{
	return m_succeed;
}


/// Set Velocity
void MotorController::setLaserMotorVelocity(double vel) {
    const  Error* err;
    err=laserMotorAmp.SetTargetVel(vel);
      if( err ) printf( "Set velocity Error: %s\n", err->toString() );
    return;
}

/// Get Velocity
void MotorController::getLaserMotorVelocity(double& vel) {
    laserMotorAmp.GetVelocityActual(vel);
	return;
}

/// Get Position
void MotorController::getLaserMotorPosition(double& pos) {

	const  Error* err;

    err = laserMotorAmp.GetPositionActual(pos);
    if( err ) printf( "get position Error: %s\n", err->toString() );
	return ;
}

void MotorController::doMove(ProfileConfigTrap &cfg){
    laserMotorAmp.DoMove(cfg);
}
