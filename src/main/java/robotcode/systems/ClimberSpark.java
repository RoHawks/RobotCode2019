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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.LocalJoystick;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;
import robotcode.pneumatics.SolenoidInterface;
import sensors.LimitSwitch;

public class ClimberSpark {

    private CANSparkMax mFrontTalon, mBackTalon;
    // private WPI_VictorSPX mFrontTalon, mBackTalon;
    // private WPI_TalonSRX mOtherBackTalon;
    private WPI_TalonSRX mDriveTalon;
    // private LimitSwitch mFrontTopLimit, mBackTopLimit, mFrontBottomLimit,
    // mBackBottomLimit; // constructor
    private LimitSwitch mFrontLegLimit, mBackLegLimit; // constructor
    private DriveTrain mDrivetrain; // constrctor
    private SolenoidInterface mShifter;
    private LocalJoystick mJoystick;

    public CANDigitalInput mFrontTopLimit, mFrontBottomLimit, mBackTopLimit, mBackBottomLimit;
    public CANEncoder mFrontEncoder, mBackEncoder;

    public ClimberSpark(CANSparkMax pFront, CANSparkMax pBack, /* WPI_TalonSRX pOtherBack, */ WPI_TalonSRX pDrive,
            SolenoidInterface pShifter, DriveTrain pDriveTrain, LocalJoystick pJoystick) {
        mFrontTalon = pFront;
        mBackTalon = pBack;
        // mOtherBackTalon = pOtherBack;
        mDriveTalon = pDrive;
        mShifter = pShifter;
        mDrivetrain = pDriveTrain;
        mJoystick = pJoystick;

        mBackTopLimit = mBackTalon.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        mBackTopLimit.enableLimitSwitch(true);

        mBackBottomLimit = mBackTalon.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        mBackBottomLimit.enableLimitSwitch(true);

        mFrontTopLimit = mFrontTalon.getForwardLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        mFrontTopLimit.enableLimitSwitch(true);

        mFrontBottomLimit = mFrontTalon.getReverseLimitSwitch(LimitSwitchPolarity.kNormallyOpen);
        mFrontBottomLimit.enableLimitSwitch(true);

        mFrontEncoder = mFrontTalon.getEncoder();
        mBackEncoder = mBackTalon.getEncoder();

        // mFrontTopLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_TOP_LIMIT,
        // 300000000);
        // mBackTopLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_TOP_LIMIT,
        // 300000000);
        // mFrontBottomLimit = new
        // LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_BOTTOM_LIMIT, 300000000);
        // mBackBottomLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_BOTTOM_LIMIT,
        // 300000000);
        mFrontLegLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_LEG_LIMIT, 300000000);
        mBackLegLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_LEG_LIMIT, 300000000);
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
                && Math.abs(mJoystick.getY(JoystickConstants.CLIMB_PROFILE)) > 0.25) {
            mFrontTalon.set(mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        } else {
            mFrontTalon.set(0);
        }

        if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.BACK)
                && Math.abs(mJoystick.getY(JoystickConstants.CLIMB_PROFILE)) > 0.25) {
            mBackTalon.set(mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        } else {
            mBackTalon.set(0);
        }

        // if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.DRIVE)
        // && Math.abs(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) > 0.5) {
        // mDriveTalon.set(ControlMode.PercentOutput,
        // -Math.signum(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE))
        // * (Math.abs(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) - 0.5));
        // } else {
        // mDriveTalon.set(0);
        // }
    }

    /**
     * Links piston and brings both legs up (robot goes down).
     * 
     * @return Whether either leg is at the top.
     */
    public boolean upLegs() {
        
        if (!mFrontTopLimit.get() && !mBackTopLimit.get()) {
            link();
            if (mFrontEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP
                    || mBackEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP) { // legs 10% from top = slow
                mFrontTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
                mBackTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
            }
            else {
                mFrontTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
                mBackTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
            }
        }

        else if(mFrontTopLimit.get() && !mBackTopLimit.get()){
            backUpLegs();
            mFrontTalon.set(0);
        }

        else if (!mFrontTopLimit.get() && mBackTopLimit.get()){
            frontUpLegs();
            mBackTalon.set(0);
        }

        else {
            mFrontTalon.set(0);
            mBackTalon.set(0);
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
       if (!mFrontBottomLimit.get() && !mBackBottomLimit.get()) {
            link();
            if (mFrontEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP
                    || mBackEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP) {
                mFrontTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
                mBackTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
            }
            else {
                mFrontTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
                mBackTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
            }
        }

        else if(mFrontBottomLimit.get() && !mBackBottomLimit.get()){
            backDownLegs();
            mFrontTalon.set(0);
        }

        else if (!mFrontBottomLimit.get() && mBackBottomLimit.get()){
            frontDownLegs();
            mBackTalon.set(0);
        }

        else {
            mFrontTalon.set(0);
            mBackTalon.set(0);
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
        mBackTalon.set(0);

        if (mFrontTopLimit.get()) {
            mFrontTalon.set(0);
            return true;
        } 
        else if (mFrontEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP) {
            mFrontTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
        } 
        else {
            mFrontTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
        }
        return false;
    }


    public boolean frontDownLegs() {
        unlink();
        mBackTalon.set(0);

        if (mFrontBottomLimit.get()) {
            mFrontTalon.set(0);
            return true;
        } 
        else if (mFrontEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP) {
            mFrontTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
        } 
        else {
            mFrontTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
        }
        return false;
    }


    public boolean backUpLegs() {
        unlink();
        mFrontTalon.set(0);

        if (mBackTopLimit.get()) {
            mBackTalon.set(0);
            return true;
        } 
        else if (mBackEncoder.getPosition() < ClimberConstants.TEN_FROM_TOP) {
            mBackTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
        } 
        else {
            mBackTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
        }
        return false;
    }


    public boolean backDownLegs() {
        unlink();
        mFrontTalon.set(0);

        if (mBackBottomLimit.get()) {
            mBackTalon.set(0);
            return true;
        } 
        else if (mBackEncoder.getPosition() > ClimberConstants.NINETY_FROM_TOP) {
            mBackTalon.set(ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
        } 
        else {
            mBackTalon.set(ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
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

    public boolean climb() {

        if (!mStepOneDone) { // go up (legs down)
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mStepOneDone = downLegs();
            SmartDashboard.putNumber("CLIMB step", 1);
        }

        else if (mStepOneDone && !mStepTwoDone) { // go forward
            mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.1, RotationalVelocity.NONE);
            mDriveTalon.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mFrontTalon.set(0);
            mBackTalon.set(0);
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
            mFrontTalon.set(0);
            mBackTalon.set(0);
            mStepFourDone = mBackLegLimit.get();
            SmartDashboard.putNumber("CLIMB step", 4);
        }

        else if (mStepOneDone && mStepTwoDone && mStepThreeDone && mStepFourDone && !mStepFiveDone) {
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mFrontTalon.set(0);
            mStepFiveDone = backUpLegs();
            SmartDashboard.putNumber("CLIMB step", 5);
        }

        else if (mStepOneDone && mStepTwoDone && mStepThreeDone && mStepFourDone && mStepFiveDone) {
            mBackTalon.set(0);
            mStepOneDone = false;
            mStepTwoDone = false;
            mStepThreeDone = false;
            mStepFourDone = false;
            mStepFiveDone = false;
            return true;
        }

        return false;
    }

}
