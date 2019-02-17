/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.JoystickConstants;
import constants.LeadscrewConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.LocalJoystick;
import robotcode.camera.Limelight;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;
import sensors.LeadscrewEncoder;


public class Leadscrew {

    // **********//
    // VARIABLES //
    // **********//

    // joysticks used
    private LocalJoystick mJoystick;
    private DriveTrain mDrivetrain;

    // leadscrew
    private WPI_TalonSRX mLeadscrew;
    private LeadscrewEncoder mEncoder;
    private LeadscrewState mLeadscrewState = LeadscrewState.IDLE; // DEFAULT TO MANUAL

    // camera
    private Limelight mHatchCamera;
    
    // ***********//
    // INITIALIZE //
    // ***********//
    public Leadscrew(WPI_TalonSRX pLeadscrew, LeadscrewEncoder pEncoder, Limelight pLimelight, LocalJoystick pJoystick, DriveTrain pDrivetrain) {
        mLeadscrew = pLeadscrew;
        mEncoder = pEncoder;
        mHatchCamera = pLimelight;
        mJoystick = pJoystick;
        mDrivetrain = pDrivetrain;
    }

    private enum LeadscrewState {
        MANUAL, CAMERA_ALIGN, IDLE, CENTER
    }


    // *********//
    // DO STUFF //
    // *********//
    
    /**
     * Control of ONLY leadscrew -- manual, camera align, or fixed loading station distance
     */
    public void enactMovement() {

        SmartDashboard.putString("LEADSCREW STATE", mLeadscrewState.toString());


        // change states
        if (mJoystick.getRawButtonPressed(6)) {
            mLeadscrewState = LeadscrewState.MANUAL;
        } 
        else if (mJoystick.getRawButtonPressed(2)) {
            mLeadscrewState = LeadscrewState.CAMERA_ALIGN;
        } 
        else if (mJoystick.getRawButtonPressed(3)){
            mLeadscrewState = LeadscrewState.CENTER;
        }
        // else {
        //     mLeadscrewState = LeadscrewState.IDLE;e
        // }

        // zero sensor
        if (!mLeadscrew.getSensorCollection().isRevLimitSwitchClosed()) {
            zero();
        }

        // do stuff
        switch (mLeadscrewState) {
            case MANUAL:
                mLeadscrew.set(ControlMode.PercentOutput, (Math.abs(mJoystick.getX()) > 0.25) ? mJoystick.getX() : 0);
                break;
            case CAMERA_ALIGN:
                centerWithCamera();
                break;
            case IDLE:
                setSpeed(0);
                break;
            case CENTER:
                setPosition(LeadscrewConstants.MIDDLE);
                break;
            default:
                setSpeed(0);
                throw new RuntimeException("Unknown leadscrew state");
            }

    }

    /**
     * @return true if close within soft limit of edges
     */
    public boolean getInSoftLimit() {
        double position = mEncoder.getDistanceInInchesFromEnd();
        return (position < LeadscrewConstants.SOFT_LIMIT
                || position > LeadscrewConstants.LENGTH - LeadscrewConstants.SOFT_LIMIT);
    }

    //ADD DIRECTION PARAMETER TO FIX THIS

    /**
     * sets raw speed of leadscrew, halves speed if close to end + is left, - is right
     * 
     * @param pSpeed percent output of motor
     */
    public void setSpeed(double pSpeed) {
        double speed = pSpeed;
        if (getInSoftLimit()) { // SLOW IT DOWN IF CLOSE TO END
            speed = Math.signum(speed) * Math.min(0.15, Math.abs(speed));
        }
        mLeadscrew.set(ControlMode.PercentOutput, speed);
    }

    public void setSpeedJoystick() {
        double joystickLocation = mJoystick.getX(JoystickConstants.LEADSCREW_PROFILE);

        double speed = Math.abs(joystickLocation) > 0.25 ? -1 * joystickLocation * Math.abs(joystickLocation) : 0;
        setSpeed(speed);
    }

    /**
     * sets position of leadscrew using talon's pid, halves P constant if close to 
     * end 0 is right, 'HatchConstants.Leadscrew.LENGTH' is left
     * 
     * @param pInchMeasurement how far from the right edge the intake should move
     */
    public void setPosition(double pInchMeasurement) {

        double goal = LeadscrewEncoder.leadscrewInchToTick(ResourceFunctions.putNumInAbsoluteRange(pInchMeasurement, 0, LeadscrewConstants.LENGTH));

        // if (getInSoftLimit()) { /*** SLOW IT DOWN IF CLOSE TO END ***/
        //     mLeadscrew.config_kP(0, LeadscrewConstants.PID.LEADSCREW_P / 2, 10);
        // } 
        // else {
            //mLeadscrew.config_kP(0, LeadscrewConstants.PID.LEADSCREW_P, 10);
        //}
        SmartDashboard.putNumber("ERROR IN LEADSCREW SET POSITION METHOD", goal - mEncoder.getRawTicks());

        mLeadscrew.set(ControlMode.Position, goal);
    }

    /**
     * aligns the leadscrew with the tape using limelight. only works in the x dimension
     */
    public void centerWithCamera() {
        if(mHatchCamera.hasTarget()) {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", true);
            double distCameraToTape = mHatchCamera.xAngleToDistance();
            double goalInches = LeadscrewConstants.MIDDLE + distCameraToTape;
            SmartDashboard.putNumber("Distance from Camera", goalInches - mEncoder.getDistanceInInchesFromEnd());
            if (Math.abs(goalInches - mEncoder.getDistanceInInchesFromEnd()) > LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
                setPosition(goalInches);
            }
        }
        else {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", false);
        }

    }

    /**
     * aligns the leadscrew with the tape using limelight + driving. only works in x dimension
     */
    public void centerWithCameraDrivetrain() {
        if (mHatchCamera.hasTarget()) {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", true);
            double distCameraToTape = mHatchCamera.xAngleToDistance();
            double goalInches = LeadscrewConstants.MIDDLE + distCameraToTape;
            SmartDashboard.putNumber("Distance from tape to leadscrew", goalInches - mEncoder.getDistanceInInchesFromEnd());
            // if the tape is farther than leadscrew's zero (0) or max distance (length)
            if(goalInches < 0){ 
                mDrivetrain.enactMovement(0, 270, LinearVelocity.NUDGE, 0, RotationalVelocity.NONE);
            }
            else if(goalInches > LeadscrewConstants.LENGTH){ 
                mDrivetrain.enactMovement(0, 90, LinearVelocity.NUDGE, 0, RotationalVelocity.NONE);
            }
            else {
                if (Math.abs(goalInches - mEncoder.getDistanceInInchesFromEnd()) > LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
                    setPosition(goalInches);
                }
            }
        } else {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", false);
        }
    }

    public void setPositionJoystick(){
        double percentGoal = (mJoystick.getX() + 1) / 2; //this should be between 0 and 1
        setPosition(percentGoal * LeadscrewConstants.LENGTH);
    }

    /**
     * sets the current position of the lead screw to be zero
     */
    public void zero() {
        mLeadscrew.setSelectedSensorPosition(0);
    }

    /**
     * when the robot starts up, drive the leadscrew to the end that zeroes it and set to zero
     */
    public void leadscrewInitialZero() {
        while (mLeadscrew.getSensorCollection().isRevLimitSwitchClosed()) {
            mLeadscrew.set(ControlMode.PercentOutput, -0.2);
            SmartDashboard.putNumber("is zeroing", System.currentTimeMillis());
            //SmartDashboard.putNumber("Talon zeroing error value", mLeadscrew.getClosedLoopError());
            //SmartDashboard.putNumber("Talon zeroing target value", mLeadscrew.getClosedLoopTarget());
        }
        mLeadscrew.set(ControlMode.PercentOutput, 0);
        // mLeadscrewTalon.configForwardSoftLimitEnable(true, 10);
		// mLeadscrewTalon.configReverseSoftLimitEnable(true, 10);
        zero();
    }

    public boolean isInRange(){
        boolean inRange = Math.abs(mEncoder.getError((int) mLeadscrew.getClosedLoopTarget())) <= LeadscrewConstants.PID.LEADSCREW_TOLERANCE;
        SmartDashboard.putBoolean("Leadscrew is in range", inRange);
        return inRange;
    }


    // ********//
    // GETTERS //
    // ********//
    public WPI_TalonSRX getTalon() {
        return mLeadscrew;
    }

    public LeadscrewState getLeadscrewState() {
        return mLeadscrewState;
    }

    public LeadscrewEncoder getLeadscrewEncoder() {
        return mEncoder;
    }
    
}   
//         else if (mJoystick.getRawButton(JoystickConstants.LeadscrewButtons.PID_TEST)){
//             mLeadscrewState = LeadscrewState.PID_TEST;
//         }

//         if (mJoystick.getRawButtonReleased(JoystickConstants.LeadscrewButtons.INCREASE_GOAL)) {
//             mPidGoalInches += 0.1;
//         } else if (mJoystick.getRawButtonReleased(JoystickConstants.LeadscrewButtons.DECREASE_GOAL)) {
//             mPidGoalInches -= 0.1;
//         }

//             case PID_TEST:
//                 setPosition(mPidGoalInches);
//                 break;
