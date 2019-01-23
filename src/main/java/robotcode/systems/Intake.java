/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import constants.JoystickConstants;
import constants.LeadscrewConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.LocalJoystick;
import robotcode.camera.Limelight;


public class Intake {

    // **********//
    // VARIABLES //
    // **********//
    private HatchIntake mHatchIntake;
    private Leadscrew mLeadscrew;
    private Limelight mLimelight;
    private LocalJoystick mJoystick;

    private IntakeState mIntakeState;

    // ***********//
    // INITIALIZE //
    // ***********//

    public Intake(HatchIntake pHatchIntake, Leadscrew pLeadscrew, Limelight pLimelight, LocalJoystick pJoystick) {
        mHatchIntake = pHatchIntake;
        mLeadscrew = pLeadscrew;
        mLimelight = pLimelight;
        mJoystick = pJoystick;
        
        mIntakeState = IntakeState.MANUAL;
    }

    private enum IntakeState {
        HATCH_INTAKE, BALL_INTAKE, HATCH_SCORE, BALL_SCORE, MANUAL
    }


    // *********//
    // DO STUFF //
    // *********//
    private boolean mDoneIntakingHatch = true;
    private boolean mDoneScoringHatch = true;
    public void enactMovement() {
        if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.INTAKE_HATCH)) {
            mIntakeState = IntakeState.HATCH_INTAKE;
        } else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.INTAKE_BALL)) {
            mIntakeState = IntakeState.BALL_INTAKE;
        } else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.SCORE_HATCH)) {
            mIntakeState = IntakeState.HATCH_SCORE;
        } else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.SCORE_BALL)) {
            mIntakeState = IntakeState.BALL_SCORE;
        } else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.MANUAL)) {
            mIntakeState = IntakeState.MANUAL;
        }

        switch (mIntakeState) {
        case HATCH_INTAKE:
            if (!mDoneIntakingHatch || mJoystick.getRawButtonReleased(JoystickConstants.IntakeButtons.INTAKE_HATCH)) {
                mDoneIntakingHatch = intakePanel();
            }
            if (mDoneIntakingHatch) {
                mIntakeState = IntakeState.MANUAL;
            }
            break;
        case BALL_INTAKE:
            intakeBall(); // to be implemented
            break;
        case HATCH_SCORE:
            if (!mDoneScoringHatch || mJoystick.getRawButtonReleased(JoystickConstants.IntakeButtons.SCORE_HATCH)) {
                mDoneScoringHatch = scorePanel();
            }
            if (mDoneScoringHatch) {
                mIntakeState = IntakeState.MANUAL;
            }
            break;
        case BALL_SCORE:
            scoreBall(); // to be implemented
            break;
        case MANUAL:
            mHatchIntake.enactMovement();
            mLeadscrew.enactMovement();
            break;
        }
    }

    private boolean mHasAlignedHatchIntake = false;
    private long mStartIntakeTimeHatch = System.currentTimeMillis();
    /**
     * intakes a hatch panel from loading station
     * @return whether the hatch panel has been fully intaken yet
     */
    public boolean intakePanel() {
        
        // start moving to position on button press
        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.LENGTH / 2) - mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("INTAKING STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAlignedHatchIntake);

        // if statements are in backwards order of steps
        // step 1: when it's aligned, set hasAligned to true, start timing, and push linear piston out
        // step 2: 500 milliseconds after step 1, intake with rotary piston
        // step 3: 1000 milliseconds after step 1, bring linear piston back
        if (mHasAlignedHatchIntake && System.currentTimeMillis() - 1000 > mStartIntakeTimeHatch) {
            SmartDashboard.putNumber("INTAKING STEP", 3);
            mHatchIntake.in();
            mHasAlignedHatchIntake = false;
            return true;
        }
        else if (mHasAlignedHatchIntake && System.currentTimeMillis() - 500 > mStartIntakeTimeHatch) {
            SmartDashboard.putNumber("INTAKING STEP", 2);
            mHatchIntake.expand();
        } 
        else if (!mHasAlignedHatchIntake && Math.abs(position - goal) < 0.15) {
            SmartDashboard.putNumber("INTAKING STEP", 1);
            mHasAlignedHatchIntake = true;
            mStartIntakeTimeHatch = System.currentTimeMillis();
            mHatchIntake.out();
        }

        return false;
    } 

    private boolean mHasAlignedHatchScore = false;
    private long mStartScoreTimeHatch = System.currentTimeMillis();
    public boolean scorePanel(){
        // start moving to position on button press
        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.LENGTH / 2) - mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("SCORING STEP", 0);
        SmartDashboard.putBoolean("scoring has aligned", mHasAlignedHatchScore);

        // if statements are in backwards order of steps
        // step 1: when it's aligned, set hasAligned to true, start timing, and push linear piston out
        // step 2: 500 milliseconds after step 1, outtake with rotary piston
        // step 3: 1000 milliseconds after step 1, bring linear piston back
        if (mHasAlignedHatchScore && System.currentTimeMillis() - 1000 > mStartScoreTimeHatch) {
            SmartDashboard.putNumber("SCORING STEP", 3);
            mHatchIntake.in();
            mHasAlignedHatchScore = false;
            return true;
        }
        else if (mHasAlignedHatchScore && System.currentTimeMillis() - 500 > mStartScoreTimeHatch) {
            SmartDashboard.putNumber("SCORING STEP", 2);
            mHatchIntake.contract(); 
        } 
        else if (!mHasAlignedHatchScore && Math.abs(position - goal) < 0.15) {
            SmartDashboard.putNumber("SCORING STEP", 1);
            mHasAlignedHatchScore = true;
            mStartScoreTimeHatch = System.currentTimeMillis();
            mHatchIntake.out();
        }

        return false;
    }

    public boolean intakeBall(){
        return true;
    }

    public boolean scoreBall(){
        return true;
    }

}
