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
import robotcode.camera.Limelight;


public class Intake {

    // **********//
    // VARIABLES //
    // **********//
    private HatchIntake mHatchIntake;
    private Leadscrew mLeadscrew;
    private Limelight mLimelight;

    public Intake(HatchIntake pHatchIntake, Leadscrew pLeadscrew, Limelight pLimelight) {
        mHatchIntake = pHatchIntake;
        mLeadscrew = pLeadscrew;
        mLimelight = pLimelight;
    }


    private boolean mHasAligned = false;
    private long mStartIntakeTime = System.currentTimeMillis();

    public boolean intakePanel() {
        mLeadscrew.centerWithCamera();
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.LENGTH / 2) - mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("INTAKING STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAligned);

        if (mHasAligned && System.currentTimeMillis() - 1000 > mStartIntakeTime) {
            mHatchIntake.in();
            SmartDashboard.putNumber("INTAKING STEP", 3);
            mHasAligned = false;
            return true;
        }
        else if (mHasAligned && System.currentTimeMillis() - 500 > mStartIntakeTime) {
            mHatchIntake.expand();
            SmartDashboard.putNumber("INTAKING STEP", 2);
        } 
        else if (!mHasAligned && Math.abs(position - goal) < 0.15) {
            mHasAligned = true;
            mStartIntakeTime = System.currentTimeMillis();
            mHatchIntake.out();
            SmartDashboard.putNumber("INTAKING STEP", 1);
        }

        return false;
    } //STAYS EXPANDED, DOESNT WAIT TO PUSH OUT

}
