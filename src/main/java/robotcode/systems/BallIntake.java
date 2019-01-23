/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import constants.BallIntakeConstants;
import constants.JoystickConstants;
import robotcode.LocalJoystick;
import robotcode.pneumatics.DoubleSolenoidReal;

/**
 * Add your docs here.
 */
public class BallIntake {

    private LocalJoystick mJoystick;
    
    private WPI_TalonSRX mBallHolder;
    private DoubleSolenoidReal mStopperPiston; 
    
    private BallIntakeState mBallIntakeState;

    public BallIntake(WPI_TalonSRX pHolder, LocalJoystick pJoystick) {
        mBallHolder = pHolder;
        mJoystick = pJoystick;
        mBallIntakeState = BallIntakeState.MANUAL;
    }

    private enum BallIntakeState{
        CARGOSHIP, ROCKET, HOLD, MANUAL
    }

    public void enactMovement() {
        if (mJoystick.getRawButtonReleased(JoystickConstants.BallGateButtons.HOLD)) {
            mBallIntakeState = BallIntakeState.HOLD;
        } else if (mJoystick.getRawButtonReleased(JoystickConstants.BallGateButtons.SCORE_CARGOSHIP)) {
            mBallIntakeState = BallIntakeState.CARGOSHIP;
        } else if (mJoystick.getRawButtonReleased(JoystickConstants.BallGateButtons.SCORE_ROCKET)) {
            mBallIntakeState = BallIntakeState.ROCKET;
        } else if (mJoystick.getRawButtonReleased(JoystickConstants.BallGateButtons.MANUAL)) {
            mBallIntakeState = BallIntakeState.MANUAL;
        }

        switch(mBallIntakeState){
            case HOLD:
                hold();
                break;
            case CARGOSHIP:
                scoreCargoship();
                break;
            case ROCKET:
                scoreRocket();
                break;
            case MANUAL:
                setVelocity((mJoystick.getY(JoystickConstants.BALL_PROFILE) > 0.25) ? mJoystick.getY(JoystickConstants.BALL_PROFILE) : 0);
                break;
        }
    }


    // *****************//
    // MOVE TO CONSTANT //
    // *****************//

    public void scoreRocket() {
        setPosition(BallIntakeConstants.Positions.HOLDER_SCORE_ROCKET_POSITION);
    }

    public void scoreCargoship() {
        setPosition(BallIntakeConstants.Positions.HOLDER_SCORE_CARGOSHIP_POSITION);
    }

    public void hold() {
        setPosition(BallIntakeConstants.Positions.HOLDER_HOLD);
    }


    // *****************//
    // GENERAL MOVEMENT //
    // *****************//
    public void setPosition(int pTicks) {
        mBallHolder.set(ControlMode.Position, pTicks + BallIntakeConstants.HOLDER_OFFSET);
    }

    public void setVelocity(double pVel) {
        mBallHolder.set(ControlMode.PercentOutput, pVel);
    }

}
