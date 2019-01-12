/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import robotcode.pneumatics.DoubleSolenoidReal;
import robotcode.pneumatics.SingleSolenoidReal;
import constants.CameraConstants;
import constants.HatchIntakeConstants;
import constants.RunConstants;
import constants.CameraConstants.Limelight;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sensors.LeadscrewEncoder;

public class HatchIntake {

    private Joystick mJoystick;

    private DoubleSolenoidReal mRotaryPiston, mLinearPiston;
    private WPI_TalonSRX mLeadscrew;
    private LeadscrewEncoder mEncoder;

    private LeadscrewState mLeadscrewState;

    public HatchIntake(DoubleSolenoidReal pRotaryPiston, WPI_TalonSRX pLeadscrewMotor, LeadscrewEncoder pEncoder,
            DoubleSolenoidReal pLinearPiston, Joystick pJoystick) {
        mLeadscrew = pLeadscrewMotor;
        mEncoder = pEncoder;
        if (RunConstants.RUNNING_PNEUMATICS) {
            mRotaryPiston = pRotaryPiston;
            mLinearPiston = pLinearPiston;
        }

        mJoystick = pJoystick;
        mLeadscrewState = LeadscrewState.MANUAL;        /*** DEFAULT TO MANUAL ***/
    }

    public enum LeadscrewState {
        MANUAL, CAMERA_ALIGN, LOADING_STATION
    }

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

    // ************//
    // LEAD SCREW //
    // ************//

    // public void enactMovement(){
    //     if(mJoystick.getRawButtonReleased(1)){
    //         mLeadscrewState = LeadscrewState.MANUAL;
    //     }
    //     else if(mJoystick.getRawButtonReleased(2)){
    //         mLeadscrewState = LeadscrewState.CAMERA_ALIGN;
    //     }
    //     else if(mJoystick.getRawButtonReleased(3)){
    //         mLeadscrewState = LeadscrewState.LOADING_STATION;
    //     }
    //     switch(mLeadscrewState){
    //         case MANUAL:
    //             mLeadscrew.set(ControlMode.PercentOutput, mJoystick.getX() * mJoystick.getX() * 0.5);
    //         case CAMERA_ALIGN:
    //             centerWithCamera();
    //         case LOADING_STATION:
    //             setPosition(HatchIntakeConstants.LeadScrew.LOADING_STATION);
    //     }
    // }



    // returns whether the leadscrew is close to the edges
    public boolean getInSoftLimit() {
        double position = LeadscrewEncoder.leadscrewTickToInch(mLeadscrew.getSelectedSensorPosition());
        return (position < HatchIntakeConstants.LeadScrew.SOFT_LIMIT
                || position > HatchIntakeConstants.LeadScrew.LENGTH - HatchIntakeConstants.LeadScrew.SOFT_LIMIT);
    }



    // sets raw speed of leadscrew, slows down if close to end
    // + is right, - is left
    public void setSpeed(double pSpeed) {
        double speed = pSpeed;
        // if (getInSoftLimit()){       /*** SLOW IT DOWN IF CLOSE TO END ***/
        //     speed /= 2;
        // }
        mLeadscrew.set(ControlMode.PercentOutput, pSpeed);
    }


    
    // sets position of leadscrew using pid, slows down if close to end
    // 0 is left, 'HatchConstants.Leadscrew.LENGTH' is right
    public void setPosition(double pInchMeasurement) {
        SmartDashboard.putNumber("Leadscrew Inch Goal", pInchMeasurement);
        double goal = LeadscrewEncoder.leadscrewInchToTick(pInchMeasurement);
        SmartDashboard.putNumber("Leadscrew Tick Goal", goal);

        // if(getInSoftLimit()){        /*** SLOW IT DOWN IF CLOSE TO END ***/
        //     mLeadscrew.config_kP(0, HatchIntakeConstants.LeadScrew.PID.LEADSCREW_P / 2, 10);
        // }
        // else{
        //     mLeadscrew.config_kP(0, HatchIntakeConstants.LeadScrew.PID.LEADSCREW_P, 10);
        // }

        mLeadscrew.set(ControlMode.Position, goal);
        SmartDashboard.putNumber("Leadscrew motor goal ticks", mLeadscrew.getClosedLoopTarget());
        SmartDashboard.putNumber("Leadscrew motor goal inches",
                LeadscrewEncoder.leadscrewTickToInch(mLeadscrew.getClosedLoopTarget()));
        SmartDashboard.putNumber("Leadscrew motor output", mLeadscrew.getMotorOutputPercent());
        SmartDashboard.putNumber("Leadscrew error", mLeadscrew.getClosedLoopError());
    }



    // aligns the leadscrew with the tape using limelight
    // currently only works with x dimension, no skew
    public void centerWithCamera() {
        double error = CameraConstants.Limelight.HEIGHT
                * NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        SmartDashboard.putNumber("Limelight error", error);
        setPosition((HatchIntakeConstants.LeadScrew.LENGTH / 2) + error);
    }



    // sets the current position of the lead screw to be zero
    // only for no offset or limit switch
    public void zero() {
        mLeadscrew.setSelectedSensorPosition(0);
    }

}

// Potentially add methods to check if hit the limit switches, if so, disable
// movement in one direction
// Have preset positions
// Get offset each time it hits limit switch to recenter it/zero it
// Depending on strategy we want, either zero it and don't have an offset, or
// have an offset and never zero it/get offset
// again each time you hit limit switch
