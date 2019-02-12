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
import constants.JoystickConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import robotcode.LocalJoystick;
import robotcode.pneumatics.SolenoidInterface;

public class Climber {

    private WPI_TalonSRX mFrontTalon, mBackTalon, mDriveTalon;
    private SolenoidInterface mShifter;
    private LocalJoystick mJoystick;

    public Climber(WPI_TalonSRX pFront, WPI_TalonSRX pBack, WPI_TalonSRX pDrive, SolenoidInterface pShifter, LocalJoystick pJoystick){
        mFrontTalon = pFront;
        mBackTalon = pBack;
        mDriveTalon = pDrive;
        mShifter = pShifter;
        mJoystick = pJoystick;
    }

    public void test() {
        if (mJoystick.getRawButtonReleased(JoystickConstants.ClimbButtons.SHIFT)) {
            mShifter.setOpposite();
        }

        if(mJoystick.getRawButton(JoystickConstants.ClimbButtons.FRONT) && Math.abs(mJoystick.getY()) > 0.25){
            mFrontTalon.set(ControlMode.PercentOutput, mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        }

        if(mJoystick.getRawButton(JoystickConstants.ClimbButtons.BACK) && Math.abs(mJoystick.getY()) > 0.25){
            mBackTalon.set(ControlMode.PercentOutput, mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        }

        if(mJoystick.getRawButton(JoystickConstants.ClimbButtons.DRIVE) && Math.abs(mJoystick.getZ()) > 0.25){
            mDriveTalon.set(ControlMode.PercentOutput, mJoystick.getZ(JoystickConstants.CLIMB_PROFILE));
        }
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
