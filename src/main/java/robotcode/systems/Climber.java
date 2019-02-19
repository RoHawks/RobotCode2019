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
import constants.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.LocalJoystick;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;
import robotcode.pneumatics.SolenoidInterface;
import sensors.LimitSwitch;

public class Climber {

    private WPI_VictorSPX mFrontTalon, mBackTalon;
    private WPI_TalonSRX mOtherBackTalon, mDriveTalon;
    private LimitSwitch mFrontTopLimit, mBackTopLimit, mFrontBottomLimit, mBackBottomLimit; // constructor
    private LimitSwitch mFrontLegLimit, mBackLegLimit; // constructor
    private DriveTrain mDrivetrain; // constrctor
    private SolenoidInterface mShifter;
    private LocalJoystick mJoystick;

    Timer t;

    public Climber(WPI_VictorSPX pFront, WPI_VictorSPX pBack, WPI_TalonSRX pOtherBack, WPI_TalonSRX pDrive, SolenoidInterface pShifter,
            DriveTrain pDriveTrain, LocalJoystick pJoystick) {
        mFrontTalon = pFront;
        mBackTalon = pBack;
        mOtherBackTalon = pOtherBack;
        mDriveTalon = pDrive;
        mShifter = pShifter;
        mDrivetrain = pDriveTrain;
        mJoystick = pJoystick;

        // mFrontTopLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_TOP_LIMIT, 300000000);
        // mBackTopLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_TOP_LIMIT, 300000000);
        // mFrontBottomLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_BOTTOM_LIMIT, 300000000);
        // mBackBottomLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_BOTTOM_LIMIT, 300000000);
        // mFrontLegLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_FRONT_LEG_LIMIT, 300000000);
        // mBackLegLimit = new LimitSwitch(Ports.ActualRobot.CLIMB_BACK_LEG_LIMIT, 300000000);
        
        t = new Timer();
    }

    public void test() {
        if (mJoystick.getRawButtonReleased(JoystickConstants.ClimbButtons.SHIFT)) {
            mShifter.setOpposite();
        }
        
        if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.FRONT)
                && Math.abs(mJoystick.getY(JoystickConstants.CLIMB_PROFILE)) > 0.25) {
            mFrontTalon.set(ControlMode.PercentOutput, mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        } else {
            mFrontTalon.set(0);
        }
        
        if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.BACK)
                && Math.abs(mJoystick.getY(JoystickConstants.CLIMB_PROFILE)) > 0.25) {
            mBackTalon.set(ControlMode.PercentOutput, mJoystick.getY(JoystickConstants.CLIMB_PROFILE));
        } else {
            mBackTalon.set(0);
        }

        // if (mJoystick.getRawButton(JoystickConstants.ClimbButtons.DRIVE)
        //         && Math.abs(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) > 0.5) {
        //     mDriveTalon.set(ControlMode.PercentOutput, -Math.signum(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE))
        //             * (Math.abs(mJoystick.getZ(JoystickConstants.CLIMB_PROFILE)) - 0.5));
        // } else {
        //     mDriveTalon.set(0);
        // }
    }

    private boolean mStartUpLegs = false;
    public boolean upLegs() {
        mShifter.set(ClimberConstants.ENABLED);
        if (!mStartUpLegs){
            mStartUpLegs = true;
            t.start();
        }

        if (mStartUpLegs && !(mFrontTopLimit.get() || mBackTopLimit.get())) {
            // if none of them are at the top, go up
            if ((t.get() * 1000) < ClimberConstants.SPEEDY_LEGS_UP_TIME) {
                mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
                mBackTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
            }
            else{
                mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
                mBackTalon.set(ControlMode.PercentOutput, ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
            }
            return false; // we're not done going up yet
        } else {
            // if any of them are at the top, stop moving
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            mBackTalon.set(ControlMode.PercentOutput, 0);
            t.stop();
            t.reset();
            mStartUpLegs = false;
            return true; // we're done going up
        }
    }

    private boolean mStartDownLegs = false;
    public boolean downLegs() {
        mShifter.set(ClimberConstants.ENABLED);

        if(!mStartDownLegs){
            mStartDownLegs = true;
            t.start();
        }

        if (mStartDownLegs && !(mFrontBottomLimit.get() || mBackBottomLimit.get())) {
            // if none of them are at the bottom, go down
            if (t.get() * 1000 < ClimberConstants.SPEEDY_LEGS_DOWN_TIME) {
                mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
                mBackTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
            }
            else{
                mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
                mBackTalon.set(ControlMode.PercentOutput, ClimberConstants.SLOW_CLIMBER_LEGS_DOWN_SPEED);
            }
            return false; // we're not done going down yet
        } else {
            // if any of them are at the bottom, stop moving
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            mBackTalon.set(ControlMode.PercentOutput, 0);
            t.stop();
            t.reset();
            mStartDownLegs = false;
            return true; // we're done going down
        }
    }

    private boolean mStartFrontUpLegs = false;
    public boolean frontUpLegs() {
        mShifter.set(ClimberConstants.DISABLED);
        mBackTalon.set(ControlMode.PercentOutput, 0);
        if(!mStartFrontUpLegs){
            mStartFrontUpLegs = true;
            t.start();
        }

        if (mStartFrontUpLegs && !mFrontTopLimit.get()) {
            // if not at the top, go up
            if ((t.get() * 1000) < ClimberConstants.SPEEDY_LEGS_UP_TIME) {
                mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
            } else {
                mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SLOW_CLIMBER_LEGS_UP_SPEED);
            }
            return false; // the front is not at the top yet
        } else {
            // if at the top, stop moving
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            t.stop();
            t.reset();
            mStartFrontUpLegs = false;
            return true; // the front is at the top now
        }
    }


    //CONTINUE CHANGING FROM HERE
    public boolean frontDownLegs() {
        mShifter.set(ClimberConstants.DISABLED);
        mBackTalon.set(ControlMode.PercentOutput, 0);
        if (!mFrontBottomLimit.get()) {
            // if not at the bottom, go down
            mFrontTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
            return false; // the front is not at the bottom yet
        } else {
            // if at the bottom, stop moving
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            return true; // the front is at the bottom now
        }
    }

    public boolean backUpLegs() {
        mShifter.set(ClimberConstants.DISABLED);
        mFrontTalon.set(ControlMode.PercentOutput, 0);
        if (!mBackTopLimit.get()) {
            // if not at the top, go up
            mBackTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_UP_SPEED);
            return false; // the back is not at the top yet
        } else {
            // if at the top, stop moving
            mBackTalon.set(ControlMode.PercentOutput, 0);
            return true; // the back is at the top now
        }
    }

    public boolean backDownLegs() {
        mShifter.set(ClimberConstants.DISABLED);
        mFrontTalon.set(ControlMode.PercentOutput, 0);
        if (!mBackBottomLimit.get()) {
            // if not at the bottom, go down
            mBackTalon.set(ControlMode.PercentOutput, ClimberConstants.SPEEDY_CLIMBER_LEGS_DOWN_SPEED);
            return false; // the back is not at the bottom yet
        } else {
            // if at the bottom, stop moving
            mBackTalon.set(ControlMode.PercentOutput, 0);
            return true; // the back is at the bottom now
        }
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

    public void climb() {
        if (!mStepOneDone) { // go up (legs down)
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mStepOneDone = downLegs();
            SmartDashboard.putNumber("CLIMB step", 1);
        } else if (mStepOneDone && !mStepTwoDone) { // go forward
            mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.1, RotationalVelocity.NONE);
            mDriveTalon.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            mBackTalon.set(ControlMode.PercentOutput, 0);
            mStepTwoDone = mFrontLegLimit.get();
            SmartDashboard.putNumber("CLIMB step", 2);
        } else if (mStepOneDone && mStepTwoDone && !mStepThreeDone) { // front leg up
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mStepThreeDone = frontUpLegs();
            SmartDashboard.putNumber("CLIMB step", 3);
        } else if (mStepOneDone && mStepTwoDone && mStepThreeDone && !mStepFourDone) {
            mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.15, RotationalVelocity.NONE);
            mDriveTalon.set(ControlMode.PercentOutput, ClimberConstants.CLIMBER_DRIVE_SPEED);
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            mBackTalon.set(ControlMode.PercentOutput, 0);
            mStepFourDone = mBackLegLimit.get();
            SmartDashboard.putNumber("CLIMB step", 4);
        } else if (mStepOneDone && mStepTwoDone && mStepThreeDone && mStepFourDone && !mStepFiveDone) {
            mDrivetrain.stop();
            mDriveTalon.set(ControlMode.PercentOutput, 0);
            mFrontTalon.set(ControlMode.PercentOutput, 0);
            mStepFiveDone = backUpLegs();
            SmartDashboard.putNumber("CLIMB step", 5);
        } else if (mStepOneDone && mStepTwoDone && mStepThreeDone && mStepFourDone && mStepFiveDone) {
            mBackTalon.set(ControlMode.PercentOutput, 0);
            mStepOneDone = false;
            mStepTwoDone = false;
            mStepThreeDone = false;
            mStepFourDone = false;
            mStepFiveDone = false;
        }
    }

    // POTENTIAL PROBLEMS:
    // 1) it should slow down near limit switch ?

}
