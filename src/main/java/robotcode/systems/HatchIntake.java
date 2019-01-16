/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import robotcode.pneumatics.DoubleSolenoidReal;
import constants.HatchIntakeConstants;
import constants.JoystickConstants;
import constants.RunConstants;

import edu.wpi.first.wpilibj.*;

public class HatchIntake {

    // **********//
    // VARIABLES //
    // **********//

    // joysticks used
    private Joystick mJoystick;

    // pistons
    private DoubleSolenoidReal 
        mRotaryPiston,
        mLinearPiston;



    // ***********//
    // INITIALIZE //
    // ***********//
    public HatchIntake(DoubleSolenoidReal pRotaryPiston, DoubleSolenoidReal pLinearPiston, Joystick pJoystick) {
        mRotaryPiston = pRotaryPiston;
        mLinearPiston = pLinearPiston;

        mJoystick = pJoystick;
    }

    // private enum HatchIntakeState {
    //     MANUAL, //combine these into intake class? name for intake class? states for hatchintake
    // }

    // **************//
    // ROTARY PISTON //
    // **************//
    public void expand() {
        if (RunConstants.RUNNING_PNEUMATICS) {
            mRotaryPiston.set(HatchIntakeConstants.RotaryPiston.OPEN);
        }
    }

    public void contract() {
        if (RunConstants.RUNNING_PNEUMATICS) {
            mRotaryPiston.set(HatchIntakeConstants.RotaryPiston.CLOSE);
        }
    }

    public void setRotaryOpposite() {
        if (RunConstants.RUNNING_PNEUMATICS) {
            mRotaryPiston.setOpposite();
        }
    }

    // **************//
    // LINEAR PISTON //
    // **************//
    public void in() {
        if (RunConstants.RUNNING_PNEUMATICS) {
            mLinearPiston.set(HatchIntakeConstants.LinearPiston.CLOSE);
        }
    }

    public void out() {
        if (RunConstants.RUNNING_PNEUMATICS) {
            mLinearPiston.set(HatchIntakeConstants.LinearPiston.OPEN);
        }
    }

<<<<<<< HEAD
    // ************//
    // LEAD SCREW  //
    // ************//

    public void enactMovement() {
        SmartDashboard.putString("leadscrewsstate", mLeadscrewState.toString());
        
        if (mJoystick.getRawButtonReleased(1)) {
            mLeadscrewState = LeadscrewState.MANUAL;
        } else if (mJoystick.getRawButtonReleased(2)) {
            mLeadscrewState = LeadscrewState.CAMERA_ALIGN;
        } else if (mJoystick.getRawButtonReleased(3)) {
            mLeadscrewState = LeadscrewState.LOADING_STATION;
        }
        
        if (mJoystick.getRawButton(4)) {
            expand();
        } else {
            contract();
        }
        if (mJoystick.getRawButton(5)) {
            out();
        } else {
            in();
        }

        if (mLeadscrew.getSensorCollection().isRevLimitSwitchClosed()) {
          zero();
        }
        
        switch (mLeadscrewState) {
        case MANUAL:
            setSpeed(Math.abs(mJoystick.getX()) > 0.25 ? -1 * mJoystick.getX() * Math.abs(mJoystick.getX()) * 0.8 : 0);
            SmartDashboard.putNumber("speed", mJoystick.getX() * mJoystick.getX() * 0.5);
            break;
        case CAMERA_ALIGN:
            centerWithCamera();
            break;
        case LOADING_STATION:
            setPosition(HatchIntakeConstants.LeadScrew.LOADING_STATION);
            break;
        default:
            setSpeed(0);
        }
        
    }

    // returns whether the leadscrew is close to the edges
    public boolean getInSoftLimit() {
        double position = mEncoder.getDistanceInInchesFromEnd();
        return (position < HatchIntakeConstants.LeadScrew.SOFT_LIMIT
                || position > HatchIntakeConstants.LeadScrew.LENGTH - HatchIntakeConstants.LeadScrew.SOFT_LIMIT);
    }

    // sets raw speed of leadscrew, slows down if close to end
    // - is right, + is left
    public void setSpeed(double pSpeed) {
        double speed = pSpeed;
        if (getInSoftLimit()) { /*** SLOW IT DOWN IF CLOSE TO END ***/
            speed /= 2;
=======
    public void setLinearOpposite() {
        if (RunConstants.RUNNING_PNEUMATICS) {
            mLinearPiston.setOpposite();
>>>>>>> 3a475c6c639caafe6353af3f0ef8b6491c96c039
        }
    }

    public void enactMovement() {
        // rotary piston
        if (mJoystick.getRawButton(JoystickConstants.HatchIntakeButtons.EXPAND_CONTRACT)) {
            setRotaryOpposite();
        }

        // linear piston
        if (mJoystick.getRawButton(JoystickConstants.HatchIntakeButtons.IN_OUT)) {
            setLinearOpposite();
        }
<<<<<<< HEAD

        mLeadscrew.set(ControlMode.Position, goal);
        SmartDashboard.putNumber("Leadscrew motor goal ticks", mLeadscrew.getClosedLoopTarget());
        SmartDashboard.putNumber("Leadscrew motor goal inches",
                LeadscrewEncoder.leadscrewTickToInch(mLeadscrew.getClosedLoopTarget()));
        SmartDashboard.putNumber("Leadscrew motor output", mLeadscrew.getMotorOutputPercent());
        SmartDashboard.putNumber("Leadscrew error", mLeadscrew.getClosedLoopError());
    }

    public void centerWithCameraWithOffset() {
        double error = mHatchCamera.xAngleToDistance(0);
        double goal = (HatchIntakeConstants.LeadScrew.LENGTH / 2) - error;
        SmartDashboard.putNumber("Distance from Camera", goal - mEncoder.getDistanceInInchesFromEnd());
        if(Math.abs(goal - mEncoder.getDistanceInInchesFromEnd()) > HatchIntakeConstants.LeadScrew.LEADSCREW_CAMERA_TOLERANCE){
            setPosition(goal);
        } 
    }

    public void zeroWithOffset(){
        mLeadscrew.setSelectedSensorPosition(HatchIntakeConstants.LeadScrew.OFFSET);
=======
>>>>>>> 3a475c6c639caafe6353af3f0ef8b6491c96c039
    }
}

// Potentially add methods to check if hit the limit switches, if so, disable
// movement in one direction
// Have preset positions
