/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import robotcode.pneumatics.SingleSolenoidReal;
import constants.HatchIntakeConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import sensors.LeadscrewEncoder;

public class HatchIntake {

    //private SingleSolenoidReal mRotaryPiston/*, mLinearPiston*/;
    private WPI_TalonSRX mLeadscrew;
    private LeadscrewEncoder mEncoder;

    public HatchIntake(/*SingleSolenoidReal pRotaryPiston,*/ WPI_TalonSRX pLeadscrewMotor, LeadscrewEncoder pEncoder
    /*, SingleSolenoidReal pLinearPiston*/) {
        //mRotaryPiston = pRotaryPiston;
        mLeadscrew = pLeadscrewMotor;
        mEncoder = pEncoder;
        //mLinearPiston = pLinearPiston;
    }

    // **************//
    // ROTARY PISTON //
    // **************//
    // public void expand() {
    //     mRotaryPiston.set(HatchIntakeConstants.RotaryPiston.OPEN);
    // }

    // public void contract() {
    //     mRotaryPiston.set(HatchIntakeConstants.RotaryPiston.CLOSE);
    // }

    // **************//
    // LINEAR PISTON //
    // **************//

/*     public void in() {
        mLinearPiston.set(HatchIntakeConstants.LinearPiston.CLOSE);
    }

    public void out() {
        mLinearPiston.set(HatchIntakeConstants.LinearPiston.OPEN);
    }
 */
    // ************//
    // LEAD SCREW //
    // ************//

    public void setSpeed(double pSpeed) {
        mLeadscrew.set(ControlMode.PercentOutput, pSpeed);
    }

    public void setPosition(double pInchMeasurement) {
        SmartDashboard.putNumber("Leadscrew Inch Goal", pInchMeasurement);
        double goal = LeadscrewEncoder.leadscrewInchToTick(pInchMeasurement);
        SmartDashboard.putNumber("Leadscrew Tick Goal", goal);

        mLeadscrew.set(ControlMode.Position, goal);
        SmartDashboard.putNumber("Leadscrew motor goal ticks", mLeadscrew.getClosedLoopTarget());
        SmartDashboard.putNumber("Leadscrew motor goal inches", LeadscrewEncoder.leadscrewTickToInch(mLeadscrew.getClosedLoopTarget()));
        SmartDashboard.putNumber("Leadscrew motor output", mLeadscrew.getMotorOutputPercent());
        SmartDashboard.putNumber("Leadscrew error", mLeadscrew.getClosedLoopError());
    }

    public void zero() {
        mLeadscrew.setSelectedSensorPosition(0);
    }

}

//Potentially add methods to check if hit the limit switches, if so, disable movement in one direction
//Have preset positions
//Get offset each time it hits limit switch to recenter it/zero it
//Depending on strategy we want, either zero it and don't have an offset, or have an offset and never zero it/get offset
//again each time you hit limit switch
