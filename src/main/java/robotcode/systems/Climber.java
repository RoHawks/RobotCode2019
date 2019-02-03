/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import robotcode.pneumatics.DoubleSolenoidReal;

public class Climber {

    private WPI_TalonSRX mFrontTalon, mBackTalon, mDriveTalon;
    private DoubleSolenoidReal mShifter;

    public Climber(WPI_TalonSRX pFront, WPI_TalonSRX pBack, WPI_TalonSRX pDrive, DoubleSolenoidReal pShifter){
        mFrontTalon = pFront;
        mBackTalon = pBack;
        mDriveTalon = pDrive;
        mShifter = pShifter;
    }

    public void up(){

    }

    public void climb(){
        
    }

}
