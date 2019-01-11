/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import robotcode.pneumatics.SingleSolenoidReal;
import constants.HatchIntakeConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HatchIntake {

//    private SingleSolenoidReal mRotaryPiston;
    private WPI_TalonSRX mLeadscrew;

    public HatchIntake(/*SingleSolenoidReal pPiston, */WPI_TalonSRX pMotor) {
        /*if ( RunConstants.RUNNING_PNEUMATICS ) {
            mRotaryPiston = pPiston;
        }*/
        mLeadscrew = pMotor;
    }

	//**********//
	// RESOURCE //
	//**********//

    public static double leadscrewTickToInch(double pTick) {
        return 1 / (HatchIntakeConstants.LeadScrew.PITCH * 4096) * pTick;
    }

    public static double leadscrewInchToTick(double pInch) {
        return HatchIntakeConstants.LeadScrew.PITCH * 4096 * pInch;
    }

    //********//
    // PISTON //
    //********//
    // private void intake(){
    //     if ( RunConstants.RUNNING_PNEUMATICS ) {
    //         mRotaryPiston.set(HatchIntakeConstants.Piston.OPEN);
    //     }
    // }

    // private void release(){
    //     if ( RunConstants.RUNNING_PNEUMATICS ) {
    //         mRotaryPiston.set(HatchIntakeConstants.Piston.CLOSE);
    //     }
    // }

    //************//
    // LEAD SCREW //
    //************//

    public void setSpeed(double pSpeed){
        mLeadscrew.set(ControlMode.PercentOutput, pSpeed);
    }

    public void set(double pInchMeasurement){
        SmartDashboard.putNumber("Leadscrew Inch Goal", pInchMeasurement);
        double goal = leadscrewInchToTick(pInchMeasurement);
        SmartDashboard.putNumber("Leadscrew Tick Goal", goal);

        mLeadscrew.set(ControlMode.Position, goal);
        SmartDashboard.putNumber("Leadscrew motor goal ticks", mLeadscrew.getClosedLoopTarget());
        SmartDashboard.putNumber("Leadscrew motor goal inches", leadscrewTickToInch(mLeadscrew.getClosedLoopTarget()));
        SmartDashboard.putNumber("Leadscrew motor output", mLeadscrew.getMotorOutputPercent());
        SmartDashboard.putNumber("Leadscrew error", mLeadscrew.getClosedLoopError());
    }

    public void zero(){
        mLeadscrew.setSelectedSensorPosition(0);
    }

    //set method that takes in an inch measurement, then sets motor to tick value w PID

}
