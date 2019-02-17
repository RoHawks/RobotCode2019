/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import constants.ClimberConstants;
import constants.JoystickConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.LocalJoystick;
import robotcode.pneumatics.SolenoidInterface;

public class Climber {

    private WPI_VictorSPX mFrontTalon, mBackTalon, mDriveTalon;
    private SolenoidInterface mShifter;
    private LocalJoystick mJoystick;

    public Climber(WPI_VictorSPX pFront, WPI_VictorSPX pBack, WPI_VictorSPX pDrive, SolenoidInterface pShifter, LocalJoystick pJoystick){
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
        // add buttons here for link and unlink 

        if(mJoystick.getRawButton(JoystickConstants.ClimbButtons.FRONT) && Math.abs(mJoystick.getY(JoystickConstants.CLIMB_PROFILE)) > 0.25){
            mFrontTalon.set(ControlMode.PercentOutput, -mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        }
        else {
            mFrontTalon.set(0);
        }

        if(mJoystick.getRawButton(JoystickConstants.ClimbButtons.BACK) && Math.abs(mJoystick.getY(JoystickConstants.CLIMB_PROFILE)) > 0.25){
            mBackTalon.set(ControlMode.PercentOutput, -mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        }
        else{
            mBackTalon.set(0);
        }

        if(mJoystick.getRawButton(JoystickConstants.ClimbButtons.DRIVE) && Math.abs(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) > 0.5){
            mDriveTalon.set(ControlMode.PercentOutput, -Math.signum(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) * (Math.abs(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) - 0.5));
        }
        else{
            mDriveTalon.set(0);
        }
    }

    public void up(){
        //if(!mFrontTalon. .getSensorCollection().isFwdLimitSwitchClosed()){
            mFrontTalon.set(ClimberConstants.ClimberSpeed);
       // }
        // else {
        //     mFrontTalon.set(0);
        // }

        //if(!mBackTalon.getSensorCollection().isFwdLimitSwitchClosed()){
            mBackTalon.set(ClimberConstants.ClimberSpeed);
        //}
        // else {
        //     mBackTalon.set(0);
        // }
    }

    public void down(){
        //if(!mFrontTalon.getSensorCollection().isRevLimitSwitchClosed()){
            mFrontTalon.set(-ClimberConstants.ClimberSpeed);
        //}
        // else {
        //     mFrontTalon.set(0);
        // }

        //if(!mBackTalon.getSensorCollection().isRevLimitSwitchClosed()){
            mBackTalon.set(-ClimberConstants.ClimberSpeed);
        //}
        // else {
        //     mBackTalon.set(0);
        // }
    }

    public void climb(){
        
    }

}
