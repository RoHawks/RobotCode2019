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
import com.revrobotics.CANDigitalInput;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;

import constants.ClimberConstants;
import constants.JoystickConstants;
import constants.Ports;
import constants.RunConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.LocalJoystick;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;
import robotcode.pneumatics.SolenoidInterface;
import sensors.LimitSwitch;

public class ClimberSpark {

    private CANSparkMax mFrontSpark, mBackSpark;
    // private WPI_VictorSPX mFrontTalon, mBackTalon;
    // private WPI_TalonSRX mOtherBackTalon;
    private WPI_TalonSRX mDriveTalon;
    // private LimitSwitch mFrontTopLimit, mBackTopLimit, mFrontBottomLimit,
    // mBackBottomLimit; // constructor
    private LimitSwitch mFrontLegLimit, mBackLegLimit; // constructor
    private DriveTrain mDrivetrain; // constrctor
    private SolenoidInterface mShifter;
    private Joystick mJoystick;

    public CANDigitalInput mFrontForwardLimit, mFrontReverseLimit, mBackForwardLimit, mBackReverseLimit;
    public CANEncoder mFrontEncoder, mBackEncoder;

    public ClimberSpark(CANSparkMax pFront, CANSparkMax pBack, /* WPI_TalonSRX pOtherBack, */ WPI_TalonSRX pDrive,
            SolenoidInterface pShifter, DriveTrain pDriveTrain, Joystick pJoystick) {
        mFrontSpark = pFront;
        mBackSpark = pBack;
        // mOtherBackTalon = pOtherBack;
        mDriveTalon = pDrive;
        mShifter = pShifter;
        mDrivetrain = pDriveTrain;
        mJoystick = pJoystick;

        mBackForwardLimit = mBackSpark.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        mBackForwardLimit.enableLimitSwitch(false);

        mBackReverseLimit = mBackSpark.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        mBackReverseLimit.enableLimitSwitch(false);

        mFrontForwardLimit = mFrontSpark.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        mFrontForwardLimit.enableLimitSwitch(false);

        mFrontReverseLimit = mFrontSpark.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyClosed);
        mFrontReverseLimit.enableLimitSwitch(false);

        mFrontEncoder = mFrontSpark.getEncoder();
        mBackEncoder = mBackSpark.getEncoder();

        // mFrontTopLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_TOP_LIMIT,
        // 300000000);
        // mBackTopLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_TOP_LIMIT,
        // 300000000);
        // mFrontBottomLimit = new
        // LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_BOTTOM_LIMIT, 300000000);
        // mBackBottomLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_BOTTOM_LIMIT,
        // 300000000);
        // mFrontLegLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_LEG_LIMIT, 300000000);
        // mBackLegLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_LEG_LIMIT, 300000000);
    }

    public void link() {
        mShifter.set(ClimberConstants.ENABLED);
    }

    public void unlink() {
        mShifter.set(ClimberConstants.DISABLED);
    }

    public void test() {
        if (mJoystick.getRawButtonReleased(JoystickConstants.ClimbButtons.SHIFT)) {
            mShifter.setOpposite();
        }

        if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.FRONT)
                && Math.abs(mJoystick.getY()) > 0.25) {
            mFrontSpark.set(mJoystick.getY()*0.85);
        } else {
            mFrontSpark.set(0);
        }

        if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.BACK)
                && Math.abs(mJoystick.getY()) > 0.25) {
            mBackSpark.set(mJoystick.getY()*0.85);
        } else {
            mBackSpark.set(0);
        }

        if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.DRIVE)
                && Math.abs(mJoystick.getZ()) > 0.5) {
            mDriveTalon.set(ControlMode.PercentOutput, -Math.signum(mJoystick.getZ())
                    * (Math.abs(mJoystick.getZ()) - 0.5));
        } else {
            mDriveTalon.set(0);
        }
    }

    /**
     * Links piston and brings both legs up (robot goes down).
     * 
     * @return Whether either leg is at the top.
     */
    public boolean upLegs() {
        
        if (!mFrontForwardLimit.get() && !mBackForwardLimit.get()) {
            link();
            if (mFrontEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP
                    || mBackEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP) { // legs 10% from top = slow
                mFrontSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
                mBackSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
            }
            else {
                mFrontSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
                mBackSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
            }
        }

        else if(mFrontForwardLimit.get() && !mBackForwardLimit.get()){
            backUpLegs();
            mFrontSpark.set(0);
        }

        else if (!mFrontForwardLimit.get() && mBackForwardLimit.get()){
            frontUpLegs();
            mBackSpark.set(0);
        }

        else {
            mFrontSpark.set(0);
            mBackSpark.set(0);
            return true;
        }

        return false;

    }

    /**
     * Links piston and brings both legs down (robot goes up).
     * 
     * @return Whether either leg is at the bottom.
     */
    public boolean downLegs() {
       if (!mFrontReverseLimit.get() && !mBackReverseLimit.get()) {
            link();
            if (mFrontEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP
                    || mBackEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP) {
                mFrontSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
                mBackSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
            }
            else {
                mFrontSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
                mBackSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
            }
        }

        else if(mFrontReverseLimit.get() && !mBackReverseLimit.get()){
            backDownLegs();
            mFrontSpark.set(0);
        }

        else if (!mFrontReverseLimit.get() && mBackReverseLimit.get()){
            frontDownLegs();
            mBackSpark.set(0);
        }

        else {
            mFrontSpark.set(0);
            mBackSpark.set(0);
            return true;
        }
        
        return false;
    }

    /**
     * Unlinks piston and brings front legs up.
     * 
     * @return Whether the front leg is at the top.
     */
    public boolean frontUpLegs() {
        unlink();
        mBackSpark.set(0);

        if (mFrontForwardLimit.get()) {
            mFrontSpark.set(0);
            return true;
        } 
        else if (mFrontEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP) {
            mFrontSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
        } 
        else {
            mFrontSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
        }
        return false;
    }


    public boolean frontDownLegs() {
        unlink();
        mBackSpark.set(0);

        if (mFrontReverseLimit.get()) {
            mFrontSpark.set(0);
            return true;
        } 
        else if (mFrontEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP) {
            mFrontSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
        } 
        else {
            mFrontSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
        }
        return false;
    }


    public boolean backUpLegs() {
        unlink();
        mFrontSpark.set(0);

        if (mBackForwardLimit.get()) {
            mBackSpark.set(0);
            return true;
        } 
        else if (mBackEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP) {
            mBackSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
        } 
        else {
            mBackSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
        }
        return false;
    }


    public boolean backDownLegs() {
        unlink();
        mFrontSpark.set(0);

        if (mBackReverseLimit.get()) {
            mBackSpark.set(0);
            return true;
        } 
        else if (mBackEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP) {
            mBackSpark.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
        } 
        else {
            mBackSpark.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
        }
        return false;
    }

    // 1) up
    // 2) forward
    // 3) front up
    // 4) forward
    // 5) back up
    private boolean mStepOneDone = false;
    private boolean mStepTwoDone = false;
    private boolean mStepThreeDone = false;
    private boolean mStepFourDone = false;
    private boolean mStepFiveDone = false;

    public boolean autoClimb() {

        if (!mStepOneDone) { // go up (legs down)
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mStepOneDone = downLegs();
            SmartDashboard.putNumber("CLIMB step", 1);
        }

        else if (mStepOneDone && !mStepTwoDone) { // go forward
            mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.1, RotationalVelocity.NONE);
            mDriveTalon.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mFrontSpark.set(0);
            mBackSpark.set(0);
            mStepTwoDone = mFrontLegLimit.get();
            SmartDashboard.putNumber("CLIMB step", 2);
        }

        else if (mStepOneDone && mStepTwoDone && !mStepThreeDone) { // front leg up
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mStepThreeDone = frontUpLegs();
            SmartDashboard.putNumber("CLIMB step", 3);
        }

        else if (mStepOneDone && mStepTwoDone && mStepThreeDone && !mStepFourDone) {
            mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.15, RotationalVelocity.NONE);
            mDriveTalon.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mFrontSpark.set(0);
            mBackSpark.set(0);
            mStepFourDone = mBackLegLimit.get();
            SmartDashboard.putNumber("CLIMB step", 4);
        }

        else if (mStepOneDone && mStepTwoDone && mStepThreeDone && mStepFourDone && !mStepFiveDone) {
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mFrontSpark.set(0);
            mStepFiveDone = backUpLegs();
            SmartDashboard.putNumber("CLIMB step", 5);
        }

        else if (mStepOneDone && mStepTwoDone && mStepThreeDone && mStepFourDone && mStepFiveDone) {
            mBackSpark.set(0);
            mStepOneDone = false;
            mStepTwoDone = false;
            mStepThreeDone = false;
            mStepFourDone = false;
            mStepFiveDone = false;
            return true;
        }

        return false;
    }

//Positive value brings the legs down
    public void manualClimb(){
        double frontClimbSpeed = 0;
        double backClimbSpeed = 0;

        if(mJoystick.getRawButton(4) && mJoystick.getRawButton(5)){
            link();
        }
        else {
            unlink();
        }

        if (mJoystick.getRawButton(4) && Math.abs(mJoystick.getY()) > 0.25) {
            if ((mJoystick.getY() > 0 && mFrontForwardLimit.get()) || (mJoystick.getY() < 0 && mFrontReverseLimit.get())){
                frontClimbSpeed = 0;
            }
            else {
                if (mJoystick.getY() < 0){
                    frontClimbSpeed = mJoystick.getY() * 0.75;
                }
                else {
                    frontClimbSpeed = mJoystick.getY() * 0.6;
                }
            }
        } 
        else {
            frontClimbSpeed = 0;
        }

        if (mJoystick.getRawButton(5) && Math.abs(mJoystick.getY()) > 0.25) {
            if ((mJoystick.getY() > 0 && mBackForwardLimit.get()) || (mJoystick.getY() < 0 && mBackReverseLimit.get())){
                backClimbSpeed = 0;
            }
            else {
                if(mJoystick.getY() < 0){
                    backClimbSpeed = mJoystick.getY() * 0.75;
                }
                else{
                    backClimbSpeed = mJoystick.getY() * 0.6;
                }
            }
        } 
        else {
            backClimbSpeed = 0;
        }
//Limit switch on the bottom is forward, limit switch on the top is reverse
        if (mJoystick.getRawButton(4) && mJoystick.getRawButton(5) && (
            (mJoystick.getY() > 0 && mFrontForwardLimit.get()) || (mJoystick.getY() < 0 && mFrontReverseLimit.get())
            || (mJoystick.getY() > 0 && mBackForwardLimit.get()) || (mJoystick.getY() < 0 && mBackReverseLimit.get()))){
            frontClimbSpeed = 0;
            backClimbSpeed = 0;
        }

        mBackSpark.set(backClimbSpeed);
        mFrontSpark.set(frontClimbSpeed);

        if(RunConstants.RUNNING_DRIVE){
            if(mJoystick.getRawButton(2)) {
                mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.2, RotationalVelocity.NONE);
            }
            else {
                mDrivetrain.stop();
            }
        }
        
        if (mJoystick.getRawButton(1)) {
            mDriveTalon.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
        } 
        else {
            mDriveTalon.set(0);
        }
    }

    public void limitSwitchLog(){
        SmartDashboard.putBoolean("Front forward limit switch", mFrontForwardLimit.get());
        SmartDashboard.putBoolean("Front reverse limit switch", mFrontReverseLimit.get());
        SmartDashboard.putBoolean("Back forward limit switch", mBackForwardLimit.get());
        SmartDashboard.putBoolean("Back reverse limit switch", mBackReverseLimit.get());
    }

}
