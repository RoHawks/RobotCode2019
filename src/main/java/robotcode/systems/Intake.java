/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import constants.IntakeConstants;
import constants.JoystickConstants;
import constants.LeadscrewConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robotcode.LocalJoystick;
import robotcode.camera.Limelight;

public class Intake {

    // **********//
    // VARIABLES //
    // **********//
    private HatchIntake mHatchIntake;
    private BallIntake mBallIntake;
    private Leadscrew mLeadscrew;
    private Limelight mLimelight;
    private LocalJoystick mJoystick;

    private IntakeState mIntakeState = IntakeState.IDLE;

    // ***********//
    // INITIALIZE //
    // ***********//

    public Intake(HatchIntake pHatchIntake, Leadscrew pLeadscrew, Limelight pLimelight, LocalJoystick pJoystick) {
        mHatchIntake = pHatchIntake;
        mLeadscrew = pLeadscrew;
        mLimelight = pLimelight;
        mJoystick = pJoystick;
    }

    private enum IntakeState {
        HATCH_INTAKE, BALL_INTAKE, HATCH_SCORE, BALL_SCORE, MANUAL, IDLE
    }

    // private Intake mInstance = null;

    // public Intake getInstance(){
    //     if (mInstance == null){
    //         mInstance = new Intake();
    //     }
    //     else return
    // }


    // *********//
    // DO STUFF //
    // *********//
    private boolean mDoneIntakingHatch = true;
    private boolean mDoneScoringHatch = true;

    public void enactMovement() {
        if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.INTAKE_HATCH)) {
            mIntakeState = IntakeState.HATCH_INTAKE;
        } 
        else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.INTAKE_BALL)) {
            mIntakeState = IntakeState.BALL_INTAKE;
        } 
        else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.SCORE_HATCH)) {
            mIntakeState = IntakeState.HATCH_SCORE;
        } 
        else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.SCORE_BALL)) {
            mIntakeState = IntakeState.BALL_SCORE;
        } 
        else if (mJoystick.getRawButton(JoystickConstants.IntakeButtons.MANUAL)) {
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
                scoreBallHigh(); // to be implemented
                break;
            case MANUAL:
                mHatchIntake.enactMovement();
                mLeadscrew.enactMovement();
                break;
            case IDLE:
                break;
            default:
                throw new RuntimeException("Unknown intake state");
        }
    }

    private boolean mHasAlignedHatchIntake = false;
    private long mStartIntakeTimeHatch = 0;

    /**
     * intakes a hatch panel from loading station
     * 
     * @return whether the hatch panel has been fully intaken yet
     */
    public boolean intakePanel() {

        // start moving to position on button press
        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError
        // works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.MIDDLE) + mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("INTAKING STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAlignedHatchIntake);

        long loadingSequenceElapsedMilliseconds = System.currentTimeMillis() - mStartIntakeTimeHatch;

        // step 0: align itself, while it's doing that contract the rotary intake
        // step 1: when it's aligned, set hasAligned to true, start timing, and push linear piston out
        // step 2: 500 milliseconds after step 1, intake with rotary piston
        // step 3: 1000 milliseconds after step 1, bring linear piston back

        if (!mHasAlignedHatchIntake && (Math.abs(position - goal) < LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE)) {
            SmartDashboard.putNumber("INTAKING STEP", 1);
            mStartIntakeTimeHatch = System.currentTimeMillis();
            mHatchIntake.out();
            mHatchIntake.contract();
            mHasAlignedHatchIntake = true;
        }
        else if (!mHasAlignedHatchIntake) {
            mHatchIntake.contract();
            // TZ this might be redundant, rotary piston should already be contracted whenever you get to this state
        }
        else if (mHasAlignedHatchIntake && (loadingSequenceElapsedMilliseconds < IntakeConstants.LoadHatchTimes.STEP_TWO)) {
            SmartDashboard.putNumber("INTAKING STEP", 2);
            mHatchIntake.expand();
        }
        else if (mHasAlignedHatchIntake && (loadingSequenceElapsedMilliseconds < IntakeConstants.LoadHatchTimes.STEP_THREE)) {
            SmartDashboard.putNumber("INTAKING STEP", 3);
            mHatchIntake.in();
            mHasAlignedHatchIntake = false;
            mStartIntakeTimeHatch = 0;
            return true;
        }

        return false;
    }

    private boolean mHasAlignedHatchScore = false;
    private long mStartScoreTimeHatch = 0;

    /**
     * scores a hatch panel
     * 
     * @return whether the hatch panel has been scored yet
     */
    public boolean scorePanel() {
        // start moving to position on button press
        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError
        // works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.MIDDLE) + mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("SCORING STEP", 0);
        SmartDashboard.putBoolean("scoring has aligned", mHasAlignedHatchScore);

        long scoringSequenceElapsedMilliseconds = System.currentTimeMillis() - mStartScoreTimeHatch;

        // step 1: when it's aligned, set hasAligned to true, start timing, and push linear piston out
        // step 2: 500 milliseconds after step 1, outtake with rotary piston
        // step 3: 1000 milliseconds after step 1, bring linear piston back

        if (!mHasAlignedHatchScore && mLeadscrew.isInRange()) { //Math.abs(position - goal) < LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE
            SmartDashboard.putNumber("SCORING STEP", 1);
            mStartScoreTimeHatch = System.currentTimeMillis();
            mHatchIntake.out();
            mHasAlignedHatchScore = true;
        }
        else if (mHasAlignedHatchScore && (scoringSequenceElapsedMilliseconds < IntakeConstants.ScoreHatchTimes.STEP_TWO)) {
            SmartDashboard.putNumber("SCORING STEP", 2);
            mHatchIntake.contract();
        }
        else if (mHasAlignedHatchScore && (scoringSequenceElapsedMilliseconds < IntakeConstants.ScoreHatchTimes.STEP_THREE)) {
            SmartDashboard.putNumber("SCORING STEP", 3);
            mHatchIntake.in();
            mStartScoreTimeHatch = 0;
            mHasAlignedHatchScore = false;
            return true;
        }
        return false;
    }


    private boolean mHasAlignedBallIntake = false;
    private long mStartIntakeTimeBall = 0;

    /**
     * does nothing yet
     * 
     * @return whether the ball has been intaken yet
     */
    public boolean intakeBall() {

        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError
        // works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.MIDDLE) - mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("BALL INTAKE STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAlignedBallScoreHigh);

        long scoringSequenceElapsedMilliseconds = System.currentTimeMillis() - mStartIntakeTimeBall;

        if (!mHasAlignedBallIntake && Math.abs(position - goal) < LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
            SmartDashboard.putNumber("BALL INTAKE STEP", 1);
            mStartIntakeTimeBall = System.currentTimeMillis();
            mBallIntake.letGo();
            mHasAlignedBallIntake = true;
        }
        else if (mHasAlignedBallIntake && scoringSequenceElapsedMilliseconds < IntakeConstants.LoadBallTimes.STEP_TWO && mBallIntake.isHoldingBall()){
            SmartDashboard.putNumber("BALL INTAKE STEP", 2);
            mBallIntake.lock();
            mStartIntakeTimeBall = 0;
            mHasAlignedBallIntake = false;
            return true; 
        }

        return false;
    }


    private boolean mHasAlignedBallScoreHigh = false;
    private long mStartScoreTimeBallHigh = 0;

    /**
     * does nothing yet
     * 
     * @return whether the ball has been scored yet
     */
    public boolean scoreBallHigh() {

        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError
        // works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.MIDDLE) - mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("BALL SCORE HIGH STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAlignedBallScoreHigh);

        long scoringSequenceElapsedMilliseconds = System.currentTimeMillis() - mStartScoreTimeBallHigh;

        if (!mHasAlignedBallScoreHigh && Math.abs(position - goal) < LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
            SmartDashboard.putNumber("BALL SCORE HIGH STEP", 1);
            mStartScoreTimeBallHigh = System.currentTimeMillis();
            mBallIntake.letGo();
            mHasAlignedBallScoreHigh = true;
        }
        else if (mHasAlignedBallScoreHigh && scoringSequenceElapsedMilliseconds < IntakeConstants.ScoreBallHighTimes.STEP_TWO){
            SmartDashboard.putNumber("BALL SCORE HIGH STEP", 2);
            mBallIntake.forward();
        }
        else if (mHasAlignedBallScoreHigh && scoringSequenceElapsedMilliseconds < IntakeConstants.ScoreBallHighTimes.STEP_THREE){
            SmartDashboard.putNumber("BALL SCORE HIGH STEP", 3);
            mBallIntake.backward();
            mBallIntake.lock();
            mStartScoreTimeBallHigh = 0;
            mHasAlignedBallScoreHigh = false;
            return true;
        }

        return false;
    }


    private boolean mHasAlignedBallScoreLow = false;
    private long mStartScoreTimeBallLow = 0;

    /**
     * does nothing yet
     * 
     * @return whether the ball has been scored yet
     */
    public boolean scoreBallLow() {
        mLeadscrew.centerWithCamera();

        // take these variables to find the error (not sure if motor.getClosedLoopError
        // works, gives weird error)
        double position = mLeadscrew.getLeadscrewEncoder().getDistanceInInchesFromEnd();
        double goal = (LeadscrewConstants.MIDDLE) - mLimelight.xAngleToDistance();

        SmartDashboard.putNumber("BALL SCORE LOW STEP", 0);
        SmartDashboard.putBoolean("has aligned", mHasAlignedBallScoreLow);

        long scoringSequenceElapsedMilliseconds = System.currentTimeMillis() - mStartScoreTimeBallLow;

        if (!mHasAlignedBallScoreLow && Math.abs(position - goal) < LeadscrewConstants.LEADSCREW_CAMERA_TOLERANCE) {
            SmartDashboard.putNumber("BALL SCORE LOW STEP", 1);
            mStartScoreTimeBallLow = System.currentTimeMillis();
            mBallIntake.release();
            mHasAlignedBallScoreLow = true;
        }
        else if (mHasAlignedBallScoreLow && scoringSequenceElapsedMilliseconds < IntakeConstants.ScoreBallLowTimes.STEP_TWO){
            SmartDashboard.putNumber("BALL SCORE LOW STEP", 2);
            mBallIntake.retain();
            mStartScoreTimeBallLow = 0;
            mHasAlignedBallScoreLow = false;
            return true;
        }
        return false;
    }

    public boolean holdingHatch() {
        mLeadscrew.setPosition(LeadscrewConstants.MIDDLE);
        mHatchIntake.in();
        mHatchIntake.expand();
        if(mLeadscrew.isInRange()){
            return true;
        }
        return false;
    }

    public boolean idle(){
        mLeadscrew.setPosition(LeadscrewConstants.MIDDLE);
        mHatchIntake.in();
        mHatchIntake.contract();
        if (RunConstants.RUNNING_BALL) {
            mBallIntake.backward();
            mBallIntake.lock();
            mBallIntake.retain();
        }
        if(mLeadscrew.isInRange()){
            return true;
        }
        return false;
    }

}
