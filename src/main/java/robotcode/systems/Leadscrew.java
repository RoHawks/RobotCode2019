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
import robotcode.LocalJoystick;
import robotcode.camera.Limelight;
import sensors.LeadscrewEncoder;

/**
 * Add your docs here.
 */
public class Leadscrew {

    // **********//
    // VARIABLES //
    // **********//

    // joysticks used
    private LocalJoystick mJoystick;

    // leadscrew
    private WPI_TalonSRX mLeadscrew;
    private LeadscrewEncoder mEncoder;
    private LeadscrewState mLeadscrewState;

    // camera
    private Limelight mHatchCamera;

    
    // ***********//
    // INITIALIZE //
    // ***********//
    public Leadscrew(WPI_TalonSRX pLeadscrew, LeadscrewEncoder pEncoder, Limelight pLimelight, LocalJoystick pJoystick) {
        mLeadscrew = pLeadscrew;
        mEncoder = pEncoder;
        mHatchCamera = pLimelight;
        mJoystick = pJoystick;

        mLeadscrewState = LeadscrewState.MANUAL; // ***** DEFAULT TO MANUAL *****//
    }

    private enum LeadscrewState {
        MANUAL, CAMERA_ALIGN, LOADING_STATION
    }



    // *********//
    // DO STUFF //
    // *********//
    
    /**
     * Control of ONLY leadscrew -- manual, camera align, or fixed loading station distance
     */
    public void enactMovement() {

        // change states
        if (mJoystick.getRawButtonReleased(JoystickConstants.LeadscrewButtons.MANUAL)) {
            mLeadscrewState = LeadscrewState.MANUAL;
        } else if (mJoystick.getRawButtonReleased(JoystickConstants.LeadscrewButtons.CAMERA_ALIGN)) {
            mLeadscrewState = LeadscrewState.CAMERA_ALIGN;
        } else if (mJoystick.getRawButtonReleased(JoystickConstants.LeadscrewButtons.LOADING_STATION)) {
            mLeadscrewState = LeadscrewState.LOADING_STATION;
        }

        // zero sensor
        if (mLeadscrew.getSensorCollection().isRevLimitSwitchClosed()) {
            zero();
        }

        // do stuff
        switch (mLeadscrewState) {
        case MANUAL:
            setSpeed(Math.abs(mJoystick.getX(JoystickConstants.LEADSCREW_PROFILE)) > 0.25 ? -1 * mJoystick.getX(JoystickConstants.LEADSCREW_PROFILE) * Math.abs(mJoystick.getX(JoystickConstants.LEADSCREW_PROFILE)) * 0.8 : 0);
            break;
        case CAMERA_ALIGN:
            centerWithCamera();
            break;
        case LOADING_STATION:
            setPosition(LeadscrewConstants.LOADING_STATION);
            break;
        default:
            setSpeed(0);
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

    /**
     * sets raw speed of leadscrew, halves speed if close to end + is left, - is right
     * 
     * @param pSpeed percent output of motor
     */
    public void setSpeed(double pSpeed) {
        double speed = pSpeed;
        if (getInSoftLimit()) { /*** SLOW IT DOWN IF CLOSE TO END ***/
            speed /= 2;
        }
        mLeadscrew.set(ControlMode.PercentOutput, speed);
    }

    /**
     * sets position of leadscrew using talon's pid, halves P constant if close to 
     * end 0 is right, 'HatchConstants.Leadscrew.LENGTH' is left
     * 
     * @param pInchMeasurement how far from the right edge the intake should move
     */
    public void setPosition(double pInchMeasurement) {
        double goal = LeadscrewEncoder.leadscrewInchToTick(pInchMeasurement);

        if (getInSoftLimit()) { /*** SLOW IT DOWN IF CLOSE TO END ***/
            mLeadscrew.config_kP(0, LeadscrewConstants.PID.LEADSCREW_P / 2, 10);
        } else {
            mLeadscrew.config_kP(0, LeadscrewConstants.PID.LEADSCREW_P, 10);
        }

        mLeadscrew.set(ControlMode.Position, goal);
    }

    /**
     * aligns the leadscrew with the tape using limelight. only works in the x
     * dimension
     */
    public void centerWithCamera() {
        double error = mHatchCamera.xAngleToDistance();
        double goal = (LeadscrewConstants.LENGTH / 2) - error;
        SmartDashboard.putNumber("Distance from Camera", goal - mEncoder.getDistanceInInchesFromEnd());
        if (Math.abs(goal - mEncoder.getDistanceInInchesFromEnd()) > LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
            setPosition(goal);
        }
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
        while (!mLeadscrew.getSensorCollection().isRevLimitSwitchClosed()) {
            mLeadscrew.set(ControlMode.PercentOutput, getInSoftLimit() ? -0.2 : -0.7);
        }
        mLeadscrew.set(ControlMode.PercentOutput, 0);
        zero();
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
