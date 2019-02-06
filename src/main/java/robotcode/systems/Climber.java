/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.ClimberConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import robotcode.pneumatics.DoubleSolenoidReal;

public class Climber {

    private WPI_TalonSRX mFrontTalon, mBackTalon, mDriveTalon;
    private DoubleSolenoidReal mShifter;
    private DigitalInput mFrontTop, mFrontBottom, mBackTop, mBackBottom;

    public Climber(WPI_TalonSRX pFront, WPI_TalonSRX pBack, WPI_TalonSRX pDrive, DoubleSolenoidReal pShifter){
        mFrontTalon = pFront;
        mBackTalon = pBack;
        mDriveTalon = pDrive;
        mShifter = pShifter;
    }

    public void up(){
        if(!mFrontTalon.getSensorCollection().isFwdLimitSwitchClosed()){
            mFrontTalon.set(ClimberConstants.ClimberSpeed);
        }
        else {
            mFrontTalon.set(0);
        }

        if(!mBackTalon.getSensorCollection().isFwdLimitSwitchClosed()){
            mBackTalon.set(ClimberConstants.ClimberSpeed);
        }
        else {
            mBackTalon.set(0);
        }
    }

    public void down(){
        if(!mFrontTalon.getSensorCollection().isRevLimitSwitchClosed()){
            mFrontTalon.set(-ClimberConstants.ClimberSpeed);
        }
        else {
            mFrontTalon.set(0);
        }

        if(!mBackTalon.getSensorCollection().isRevLimitSwitchClosed()){
            mBackTalon.set(-ClimberConstants.ClimberSpeed);
        }
        else {
            mBackTalon.set(0);
        }
    }

    public void climb(){
        
    }

}
