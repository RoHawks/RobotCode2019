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
import resource.MovingAverage;
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
        if (mJoystick.getRawButtonPressed(26)) {
            mLeadscrewState = LeadscrewState.MANUAL;
        } 
        else if (mJoystick.getRawButtonPressed(22)) {
            mLeadscrewState = LeadscrewState.CAMERA_ALIGN;
        } 
        else if (mJoystick.getRawButtonPressed(23)){
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
                //centerWithCameraDrivetrain();
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
        mLeadscrew.set(ControlMode.Position, goal);
    }

    //pass moving average in constructor
    //MovingAverage mCameraAverage = new MovingAverage(5);
    /**
     * aligns the leadscrew with the tape using limelight. only works in the x dimension
     */
    public void centerWithCamera() {
        if(mHatchCamera.hasTarget()) {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", true);
            double distCameraToTape = mHatchCamera.xAngleToDistance();
            //mCameraAverage.addNumber(distCameraToTape);
            //distCameraToTape = mCameraAverage.getAverage;
            double goalInches = LeadscrewConstants.MIDDLE + distCameraToTape;
            //SmartDashboard.putNumber("Distance from Camera", goalInches - mEncoder.getDistanceInInchesFromEnd());
            if (Math.abs(goalInches - mEncoder.getDistanceInInchesFromEnd()) > LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
                setPosition(goalInches);
            }
        }
        else {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", false);
        }
        //mCameraAverage.clear();
    }

    private long mTimeStartDriveAlign = 0;
    private boolean mStartDriveAlign = false;
    private long mTimeMoveBack = 450;

    private boolean mStartDriveScore = false;
    private long mTimeStartDriveScore = 0;
    private long mTimeMoveForward = 500;

    private MovingAverage mAvg = new MovingAverage(5);
    /**
     * aligns the leadscrew with the tape using limelight + driving. only works in x dimension
     */
    public void centerWithCameraDrivetrain() { // absolute cancer

        if (mHatchCamera.hasTarget()) {

            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", true);
            double distCameraToTape = mHatchCamera.xAngleToDistance();
            //TODO check moving average
            mAvg.addNumber(distCameraToTape);
            distCameraToTape = mAvg.getAverage();
            double goalInches = LeadscrewConstants.MIDDLE + distCameraToTape;
            // if the tape is farther than 1 inch from leadscrew's zero (0)
            if (goalInches < 3) {
                if (!mStartDriveAlign) {
                    // if you haven't started this drive align process, set mStartDriveAlign to true
                    // and start timing
                    mStartDriveAlign = true;
                    mTimeStartDriveAlign = System.currentTimeMillis();
                } 

                if (mStartDriveAlign && System.currentTimeMillis() - mTimeStartDriveAlign < mTimeMoveBack) {
                    // if you've started drive align process, move back for TIME
                    mDrivetrain.enactMovement(0, 180, LinearVelocity.NORMAL, 0.5, RotationalVelocity.NONE);
                }

                else {
                    // if you've started drive align process and have moved back already, then start
                    // moving towards the tape. when you reach it, you'll go to the else statement
                    mDrivetrain.enactMovement(0, 270, LinearVelocity.NORMAL, 0.4, RotationalVelocity.NONE);
                }
            }
            // if the tape is farther than 1inch from leadscrew's max distance (length)
            else if (goalInches > LeadscrewConstants.LENGTH - 3) {
                // if you haven't started this drive align process, set mStartDriveAlign to true
                // and start timing
                if (!mStartDriveAlign) {
                    mStartDriveAlign = true;
                    mTimeStartDriveAlign = System.currentTimeMillis();
                } 
                
                if (mStartDriveAlign && System.currentTimeMillis() - mTimeStartDriveAlign < mTimeMoveBack) {
                    // if you've started drive align process, move back for TIME
                    mDrivetrain.enactMovement(0, 180, LinearVelocity.NORMAL, 0.5, RotationalVelocity.NONE);
                }

                else { // if you've started drive align process and have moved back already, then start
                         // moving towards the tape. when you reach it, you'll go to the else statement
                    mDrivetrain.enactMovement(0, 90, LinearVelocity.NORMAL, 0.4, RotationalVelocity.NONE);
                }
            } 
            
            else { // you'll reach here after the if and else statement move you over the tape
                mStartDriveAlign = false; // then you're done with the drive align part -- reset that to false
                mTimeStartDriveAlign = 0; // when you're done with the drive align part -- reset time to 0
                if (!mStartDriveScore) { // if you haven't started the drive score process, start that
                    mTimeStartDriveScore = System.currentTimeMillis();
                    mStartDriveScore = true;
                }
                else if (mStartDriveScore && System.currentTimeMillis() - mTimeStartDriveScore < mTimeMoveForward) {
                    // when you want to score process, move forward to compensate for backwards
                    // movement above / ram into the ship
                    mDrivetrain.enactMovement(0, 0, LinearVelocity.NORMAL, 0.5, RotationalVelocity.NONE);
                } 
                else { // once you've moved forward, align the leadscrew
                    mDrivetrain.stop();
                    // if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)){
                    //     double joystickPosition = mJoystick.getX();
                    //     if (Math.abs(joystickPosition) < 0.2){
                    //         setPosition(LeadscrewConstants.MIDDLE);
                    //     }
                    // }
                    // else {
                        if (Math.abs(goalInches - mEncoder.getDistanceInInchesFromEnd()) > LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
                            setPosition(goalInches);
                        } 
                        else { // once the leadscrew error is less than the tolerance, you're done. reset
                                 // everything
                            //mTimeStartDriveScore = 0;
                           // mStartDriveScore = false;
                           //mAvg.clear();
                        }
                   // }
                }
            }
        } 
        else {
            SmartDashboard.putBoolean("TAPE TARGET ACQUIRED", false);
        }
    }

    public void setPositionJoystick(){
        setPosition(mJoystick.getX());
        // double percentGoal = (mJoystick.getX() + 1) / 2; //this should be between 0 and 1
        // setPosition(percentGoal * LeadscrewConstants.LENGTH);
    }

    /**
     * sets the current position of the lead screw to be zero
     */
    public void zero() {
        mLeadscrew.setSelectedSensorPosition(0);
    }

    private long timeStartedZeroing = 0;
    /**
     * when the robot starts up, drive the leadscrew to the end that zeroes it and set to zero
     */
    public void leadscrewInitialZero() {

        timeStartedZeroing = System.currentTimeMillis();
        

        while (mLeadscrew.getSensorCollection().isRevLimitSwitchClosed()) {
           // if(System.currentTimeMillis() - timeStartedZeroing < 5000){
                mLeadscrew.set(ControlMode.PercentOutput, -0.3);
        
                SmartDashboard.putBoolean("in lead screw initial zero", true);
                if(mJoystick.getRawButtonReleased(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)){
                    break;
                }
            // }
            // else {
            //     mLeadscrew.set(ControlMode.PercentOutput, -0.4);
            // }
        }
        SmartDashboard.putBoolean("in lead screw initial zero", false);

        mLeadscrew.set(ControlMode.PercentOutput, 0);
        mLeadscrew.configForwardSoftLimitEnable(false, 10);
		mLeadscrew.configReverseSoftLimitEnable(false, 10);
        zero();
    }

    public boolean isInRange(){
        boolean inRange = Math.abs(mEncoder.getError((int) mLeadscrew.getClosedLoopTarget())) <= LeadscrewConstants.PID.LEADSCREW_TOLERANCE;
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
