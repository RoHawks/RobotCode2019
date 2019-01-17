/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import constants.CameraConstants;
import constants.LeadscrewConstants;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Intake {

    // **********//
    // VARIABLES //
    // **********//
    private HatchIntake mHatchIntake;
    private Leadscrew mLeadscrew;

    public Intake(HatchIntake pHatchIntake, Leadscrew pLeadscrew) {
        mHatchIntake = pHatchIntake;
        mLeadscrew = pLeadscrew;
    }


    private boolean mHasAligned = false;
    private long mStartIntakeTime = System.currentTimeMillis();
    public boolean intakePanel() {
        mLeadscrew.centerWithCamera();
        double position = mLeadscrew.getTalon().getSelectedSensorPosition() / (4096 * 12);
        double goal = (LeadscrewConstants.LENGTH / 2) - CameraConstants.LimelightConstants.HEIGHT * Math.tan(Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0)));
        SmartDashboard.putNumber("INTAKING STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAligned);
        if (mHasAligned && System.currentTimeMillis() - 1000 > mStartIntakeTime) {
            mHatchIntake.in();
            SmartDashboard.putNumber("INTAKING STEP", 3);
            mHasAligned = false;
            return true;
        } else if (mHasAligned && System.currentTimeMillis() - 500 > mStartIntakeTime) {
            mHatchIntake.expand();
            SmartDashboard.putNumber("INTAKING STEP", 2);
        } else if (!mHasAligned && Math.abs(position - goal) < 500) {
            mHasAligned = true;
            mStartIntakeTime = System.currentTimeMillis();
            mHatchIntake.out();
            SmartDashboard.putNumber("INTAKING STEP", 1);
        } 
        return false;
    } //STAYS EXPANDED, DOESNT WAIT TO PUSH OUT

}
