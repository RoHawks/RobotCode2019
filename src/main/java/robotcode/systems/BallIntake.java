/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import constants.BallIntakeConstants;
import robotcode.pneumatics.DoubleSolenoidReal;

/**
 * Add your docs here.
 */
public class BallIntake {

    private DoubleSolenoidReal mLeftPiston, mRightPiston;

    public BallIntake(DoubleSolenoidReal pLeftPiston, DoubleSolenoidReal pRightPiston) {
        mLeftPiston = pLeftPiston;
        mRightPiston = pRightPiston;
    }

    public void open() {
        mLeftPiston.set(BallIntakeConstants.LEFT_OPEN);
        mRightPiston.set(BallIntakeConstants.RIGHT_OPEN);
    }

    public void close() {
        mLeftPiston.set(BallIntakeConstants.LEFT_CLOSE);
        mRightPiston.set(BallIntakeConstants.RIGHT_CLOSE);
    }

}
