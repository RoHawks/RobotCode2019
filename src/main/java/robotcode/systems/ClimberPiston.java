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
import constants.Ports;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.Relay.Direction;
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

    private Relay mFrontRelay, mBackRelayBrake, mBackRelayThrough;

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

        mFrontRelay = new Relay(Ports.ActualRobot.CLIMB_FRONT_RELAY, Direction.kForward);
        mBackRelayBrake = new Relay(Ports.ActualRobot.CLIMB_BACK_RELAY_BRAKE, Direction.kForward);
        mBackRelayThrough = new Relay(Ports.ActualRobot.CLIMB_BACK_RELAY_THROUGH, Direction.kForward);
    }

    public void enactMovement(){
        if(mJoystick.getRawButton(4)){
            mFrontRelay.set(edu.wpi.first.wpilibj.Relay.Value.kForward); // brake
        }
        else{
            mFrontRelay.set(edu.wpi.first.wpilibj.Relay.Value.kOff); // not brake
        }

        if(mJoystick.getRawButton(5)){
            mBackRelayBrake.set(edu.wpi.first.wpilibj.Relay.Value.kOff);
            mBackRelayThrough.set(edu.wpi.first.wpilibj.Relay.Value.kForward);  // not brake
        }
        else{
            mBackRelayBrake.set(edu.wpi.first.wpilibj.Relay.Value.kForward);
            mBackRelayThrough.set(edu.wpi.first.wpilibj.Relay.Value.kOff); // brake
        }

        if(mJoystick.getRawButtonReleased(2)){
            mFrontPiston.setOpposite();
        }
        if(mJoystick.getRawButtonReleased(3)){
            mBackPiston.setOpposite();
        }
    }


    public void bothLegsDown() {
        extendFrontPiston();
        extendBackPiston();
    }

    public void bothLegsUp() {
        retractFrontPiston();
        retractBackPiston();
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
        else if (mJoystick.getRawButton(6)) {
            manualClimb3();
        }
        else if (mJoystick.getRawButtonReleased(6)){
            mBackBreak.set(ClimberConstants.BACK_THROUGH);
            mFrontBreak.set(ClimberConstants.FRONT_THROUGH);
            climb3State = ALL_GOOD;
        }

        SmartDashboard.putString("CLIMB front", mFrontRelay.get().toString());
        SmartDashboard.putString("CLIMB back brake", mBackRelayBrake.get().toString());
        SmartDashboard.putString("CLIMB back through", mBackRelayThrough.get().toString());



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

    public double GetTilt(){
        return mNavX.getPitch();
    }


    final int ALL_GOOD = 0;
    final int CORRECTING_FRONT_TOO_HIGH = 1;
    int AFTER_FRONT_TOO_HIGH_CORRECTION;
    final int CORRECTING_BACK_TOO_HIGH = 2;
    int AFTER_CORRECTING_BACK_TOO_HIGH;
    int climb4State = ALL_GOOD;
    long stateStart;
    final long AFTER_TIME = 200;
    final long CORRECTION_TIME = 100;

    public void manualClimb4() {

        double tooTilted = 3;
        if (climb4State == ALL_GOOD) {
            extendFrontPiston();
            extendBackPiston();
            if (GetTilt() < -tooTilted) {
                climb4State = CORRECTING_BACK_TOO_HIGH;
                stateStart = System.currentTimeMillis();
            }
            if (GetTilt() > tooTilted) {
                climb4State = CORRECTING_FRONT_TOO_HIGH;
                stateStart = System.currentTimeMillis();
            }
        } else if (climb4State == CORRECTING_FRONT_TOO_HIGH) {
            brakeFrontPiston();
            extendBackPiston();
            if (System.currentTimeMillis() - stateStart > CORRECTION_TIME) {
                climb4State = AFTER_FRONT_TOO_HIGH_CORRECTION;
                stateStart = System.currentTimeMillis();
            }
        } else if (climb4State == CORRECTING_BACK_TOO_HIGH) {
            brakeBackPiston();
            extendFrontPiston();
            if (System.currentTimeMillis() - stateStart > CORRECTION_TIME) {
                climb4State = AFTER_CORRECTING_BACK_TOO_HIGH;
                stateStart = System.currentTimeMillis();
            }
        } else if (climb4State == AFTER_FRONT_TOO_HIGH_CORRECTION) {
            extendFrontPiston();
            extendBackPiston();
            if (System.currentTimeMillis() - stateStart > AFTER_TIME) {
                if (GetTilt() < -tooTilted) {
                    climb4State = CORRECTING_BACK_TOO_HIGH;
                    stateStart = System.currentTimeMillis();
                } else if (GetTilt() > tooTilted) {
                    climb4State = CORRECTING_FRONT_TOO_HIGH;
                    stateStart = System.currentTimeMillis();
                } else {
                    climb4State = ALL_GOOD;
                    stateStart = System.currentTimeMillis();
                }
            }
        } else {
            extendFrontPiston();
            brakeBackPiston();
            if (System.currentTimeMillis() - stateStart > AFTER_TIME) {
                if (GetTilt() < -tooTilted) {
                    climb4State = CORRECTING_BACK_TOO_HIGH;
                    stateStart = System.currentTimeMillis();
                } else if (GetTilt() > tooTilted) {
                    climb4State = CORRECTING_FRONT_TOO_HIGH;
                    stateStart = System.currentTimeMillis();
                } else {
                    climb4State = ALL_GOOD;
                    stateStart = System.currentTimeMillis();
                }
            }
        }

    }

    int climb3State = ALL_GOOD;
    public void manualClimb3(){
        double frontTilted = 3;
        double backTilted = -3;
        double currentTilt = GetTilt();
        SmartDashboard.putNumber("ClimberTilt", currentTilt);
        switch(climb3State) {
            case ALL_GOOD:
                if(currentTilt > frontTilted) {
                    SmartDashboard.putString("ClimberState", "Front Too High");
                    brakeFrontPiston();
                    extendBackPiston();
                    climb3State = CORRECTING_FRONT_TOO_HIGH;
                }
                else if(currentTilt < backTilted) {
                    SmartDashboard.putString("ClimberState", "Back Too High");
                    extendFrontPiston();
                    brakeBackPiston();
                    climb3State = CORRECTING_BACK_TOO_HIGH;
                }
                else {
                    SmartDashboard.putString("ClimberState", "Looking Good!");
                    extendFrontPiston();
                    extendBackPiston();
                }
                break;
            case CORRECTING_FRONT_TOO_HIGH:
                if(currentTilt < backTilted) {
                    SmartDashboard.putString("ClimberState", "Back Too High");
                    extendFrontPiston();
                    brakeBackPiston();
                    climb3State = CORRECTING_BACK_TOO_HIGH;
                }
                break;
            case CORRECTING_BACK_TOO_HIGH:
                if(currentTilt > frontTilted) {
                    SmartDashboard.putString("ClimberState", "Front Too High");
                    brakeFrontPiston();
                    extendBackPiston();
                    climb3State = CORRECTING_FRONT_TOO_HIGH;
                }
                break;
        }
    }    

    public void brakeFrontPiston()
    {
        mFrontRelay.set(edu.wpi.first.wpilibj.Relay.Value.kForward);
    }

    public void brakeBackPiston()
    {
        mBackRelayBrake.set(edu.wpi.first.wpilibj.Relay.Value.kForward);
        mBackRelayThrough.set(edu.wpi.first.wpilibj.Relay.Value.kOff);
    }

    public void extendFrontPiston()
    {
        mFrontRelay.set(edu.wpi.first.wpilibj.Relay.Value.kOff);
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_DOWN);
    }

    
    public void extendBackPiston()
    {
        mBackRelayBrake.set(edu.wpi.first.wpilibj.Relay.Value.kOff);
        mBackRelayThrough.set(edu.wpi.first.wpilibj.Relay.Value.kForward);
        mBackPiston.set(ClimberConstants.BACK_LEGS_DOWN);
    }

    public void retractFrontPiston()
    {
        mFrontRelay.set(edu.wpi.first.wpilibj.Relay.Value.kOff);
        mFrontPiston.set(ClimberConstants.FRONT_LEGS_UP);
    }

    
    public void retractBackPiston()
    {
        mBackRelayBrake.set(edu.wpi.first.wpilibj.Relay.Value.kOff);
        mBackRelayThrough.set(edu.wpi.first.wpilibj.Relay.Value.kForward);
        mBackPiston.set(ClimberConstants.BACK_LEGS_UP);
    }

    public void autoClimb() {
        //implement this if manual climb works lol
    }

}