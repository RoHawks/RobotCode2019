/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import constants.ClimberConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;
import robotcode.pneumatics.SolenoidInterface;

public class ClimberPiston {

    private SolenoidInterface mBackPiston, mFrontPiston, mBackBreak, mFrontBreak;
    private WPI_TalonSRX mDriveTalon0, mDriveTalon1;
    private DriveTrain mDriveTrain;
    private Joystick mJoystick;
    private AHRS mNavX;

    public ClimberPiston(SolenoidInterface pBackPiston, SolenoidInterface pFrontPiston, WPI_TalonSRX pDriveTalon0,
            WPI_TalonSRX pDriveTalon1, DriveTrain pDriveTrain, Joystick pJoystick, 
            SolenoidInterface pBackBreak, SolenoidInterface pFrontBreak, AHRS pNavX) {
        mBackPiston = pBackPiston;
        mFrontPiston = pFrontPiston;
        mDriveTalon0 = pDriveTalon0;
        mDriveTalon1 = pDriveTalon1;
        mDriveTrain = pDriveTrain;
        mJoystick = pJoystick;
        mBackBreak = pBackBreak;
        mFrontBreak = pFrontBreak;
        mNavX = pNavX;
    }

    private Value frontPistonValue = ClimberConstants.FRONT_LEGS_UP;
    private Value backPistonValue = ClimberConstants.BACK_LEGS_UP;

    public void enactMovement() {
        if (mJoystick.getRawButtonReleased(1)) {
            frontPistonValue = ClimberConstants.FRONT_LEGS_UP;
        }
        if (mJoystick.getRawButtonReleased(2)) {
            frontPistonValue = ClimberConstants.FRONT_LEGS_DOWN;
        }
        if (mJoystick.getRawButtonReleased(3)) {
            backPistonValue = ClimberConstants.BACK_LEGS_UP;
        }
        if (mJoystick.getRawButtonReleased(4)) {
            backPistonValue = ClimberConstants.BACK_LEGS_DOWN;
        }
        if (mJoystick.getRawButton(5)) {
            mDriveTalon0.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mDriveTalon1.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mDriveTrain.enactMovement(0, 0, LinearVelocity.ANGLE_ONLY, 0, RotationalVelocity.NONE);
        }

        else {
            mDriveTalon0.set(0);
            mDriveTalon1.set(0);
        }
    }

    public void frontLegsDown() {
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_DOWN);
    }

    public void frontLegsUp() {
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_UP);
    }

    public void backLegsDown() {
        mBackPiston.set(ClimberConstants.BACK_LEGS_DOWN);
    }

    public void backLegsUp() {
        mBackPiston.set(ClimberConstants.BACK_LEGS_UP);
    }

    public void bothLegsDown() {
        backLegsDown();
        frontLegsDown();
    }

    public void bothLegsUp() {
        backLegsUp();
        frontLegsUp();
    }

    public void manualClimb() {

        boolean frontButton = mJoystick.getRawButtonReleased(4);
        boolean backButton = mJoystick.getRawButtonReleased(5);

        if (frontButton || backButton) {
            if (frontButton) {
                retractFrontPiston();
            }
            if (backButton) {
                retractBackPiston();
            }
        } 
        else if (mJoystick.getRawButtonReleased(3)) {
            bothLegsDown();
        } 
        else if (mJoystick.getRawButton(6)) {
            manualClimb4();
        }

        if(mJoystick.getRawButtonReleased(7)){
            mBackBreak.setOpposite();
        }
        if(mJoystick.getRawButtonReleased(8)){
            mFrontBreak.setOpposite();
        }



        if (RunConstants.RUNNING_DRIVE) {
            if (mJoystick.getRawButton(2)) {
                mDriveTrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.2, RotationalVelocity.NONE);
            } 
            else {
                mDriveTrain.stop();
            }
        }

        if (mJoystick.getRawButton(1)) {
            mDriveTalon0.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mDriveTalon1.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mDriveTrain.enactMovement(0, 0, LinearVelocity.ANGLE_ONLY, 0, RotationalVelocity.NONE);
        }

        else {
            mDriveTalon0.set(0);
            mDriveTalon1.set(0);
        }
    }

    public double GetTilt()
    {
        return mNavX.getPitch();
        //return mNavX.getRoll();
    }


    int ALL_GOOD = 0;
    int CORRECTING_FRONT_TOO_HIGH;
    int AFTER_FRONT_TOO_HIGH_CORRECTION;
    int CORRECTING_BACK_TOO_HIGH;
    int AFTER_CORRECTING_BACK_TOO_HIGH;
    int climb4State = ALL_GOOD;
    long stateStart;
    long AFTER_TIME = 200;
    long CORRECTION_TIME = 100;

public void manualClimb4()
{
    
    double tooTilted = 3;
    if(climb4State == ALL_GOOD)
    {
        extendFrontPiston();
        extendBackPiston();
        if(GetTilt() > tooTilted)
        {
            climb4State = CORRECTING_BACK_TOO_HIGH;
            stateStart = System.currentTimeMillis();
        }
         if(GetTilt() < -tooTilted)
         {
            climb4State = CORRECTING_FRONT_TOO_HIGH;
            stateStart = System.currentTimeMillis();             
         }
    }
    else if(climb4State == CORRECTING_FRONT_TOO_HIGH)
    {
        breakFrontPiston();
        extendBackPiston();
        if(System.currentTimeMillis() - stateStart > CORRECTION_TIME)
        {
            climb4State = AFTER_FRONT_TOO_HIGH_CORRECTION;
            stateStart = System.currentTimeMillis();
        }
    }
    else if(climb4State == CORRECTING_BACK_TOO_HIGH)
    {
        breakBackPiston();
        extendFrontPiston();
        if(System.currentTimeMillis() - stateStart > CORRECTION_TIME)
        {
            climb4State = AFTER_CORRECTING_BACK_TOO_HIGH;
            stateStart = System.currentTimeMillis();
        }
    }
    else if(climb4State == AFTER_FRONT_TOO_HIGH_CORRECTION)
    {
        extendFrontPiston();
        extendBackPiston();
        if(System.currentTimeMillis() - stateStart > AFTER_TIME)
        {
            if(GetTilt() > tooTilted)
            {
                climb4State = CORRECTING_BACK_TOO_HIGH;
                stateStart = System.currentTimeMillis();
            }
            else if(GetTilt() < -tooTilted)
             {
                climb4State = CORRECTING_FRONT_TOO_HIGH;
                stateStart = System.currentTimeMillis();             
             }
             else
             {
                climb4State = ALL_GOOD;
                stateStart = System.currentTimeMillis();
             }
        }
    }
    else
    {
        extendFrontPiston();
        breakBackPiston();
        if(System.currentTimeMillis() - stateStart > AFTER_TIME)
        {
            if(GetTilt() > tooTilted)
            {
                climb4State = CORRECTING_BACK_TOO_HIGH;
                stateStart = System.currentTimeMillis();
            }
            else if(GetTilt() < -tooTilted)
             {
                climb4State = CORRECTING_FRONT_TOO_HIGH;
                stateStart = System.currentTimeMillis();             
             }
             else
             {
                climb4State = ALL_GOOD;
                stateStart = System.currentTimeMillis();
             }
        }
    }

}


    public void manualClimb2(){
        double tooTilted = 3;
        SmartDashboard.putNumber("ClimberTilt", GetTilt());
        if(GetTilt() > tooTilted)
        {
            //Front has gotten too high
            SmartDashboard.putString("ClimberState", "Front Too High");
            breakFrontPiston();
            extendBackPiston();
        }
        else if(GetTilt() < -tooTilted)
        {
            
            SmartDashboard.putString("ClimberState", "Back Too High");
            extendFrontPiston();
            breakBackPiston();
        }
        else{
            
            SmartDashboard.putString("ClimberState", "Looking Good!");
            extendFrontPiston();
            extendBackPiston();
        }
    }

    public void manualClimb3(){
        double tooTilted = 10;
        double restartMoving = 3;
        boolean wasJustCorrecting = false;
        SmartDashboard.putNumber("ClimberTilt", GetTilt());
        if(GetTilt() > tooTilted)
        {
            SmartDashboard.putString("ClimberState", "Front Too High");
            breakFrontPiston();
            extendBackPiston();
            wasJustCorrecting = true;
        }
        else if(GetTilt() > restartMoving && GetTilt() < tooTilted){
            if(wasJustCorrecting){
                breakFrontPiston();
                extendBackPiston();
            }
            else{
            }
        }
        else if(GetTilt() < -tooTilted)
        {
            SmartDashboard.putString("ClimberState", "Back Too High");
            extendFrontPiston();
            breakBackPiston();
            wasJustCorrecting = true;
        }
        else{
            SmartDashboard.putString("ClimberState", "Looking Good!");
            extendFrontPiston();
            extendBackPiston();
            wasJustCorrecting = false;
        }
    }    

    public void breakFrontPiston()
    {
        mFrontBreak.set(ClimberConstants.FRONT_BREAK);
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_DOWN);
    }

    public void breakBackPiston()
    {
        mBackBreak.set(ClimberConstants.BACK_BREAK);
        mBackPiston.set(ClimberConstants.BACK_LEGS_DOWN);
    }

    public void extendFrontPiston()
    {
        mFrontBreak.set(ClimberConstants.FRONT_THROUGH);
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_DOWN);
    }

    
    public void extendBackPiston()
    {
        mBackBreak.set(ClimberConstants.BACK_THROUGH);
        mBackPiston.set(ClimberConstants.BACK_LEGS_DOWN);
    }

    public void retractFrontPiston()
    {
        mFrontBreak.set(ClimberConstants.FRONT_THROUGH);
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_UP);
    }

    
    public void retractBackPiston()
    {
        mBackBreak.set(ClimberConstants.BACK_THROUGH);
        mBackPiston.set(ClimberConstants.BACK_LEGS_UP);
    }

    public void autoClimb() {
        //implement this if manual climb works lol
    }

}