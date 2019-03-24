/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package constants;

public class IntakeConstants {

    public static final long 
        MOVE_BACK_TIME = 500,
        BALL_MOVE_OUT_TIME = 1300;

    public static class ScoreHatchTimes {
        public static final long
            STEP_TWO = HatchIntakeConstants.LinearPiston.OUT_TIME,
            STEP_THREE = HatchIntakeConstants.RotaryPiston.CONTRACT_TIME + STEP_TWO,
            STEP_FOUR = HatchIntakeConstants.LinearPiston.IN_TIME + STEP_THREE,
            STEP_FIVE = MOVE_BACK_TIME + STEP_FOUR;
    }

    public static class LoadHatchTimes {
        public static final long
            STEP_TWO = Math.max(HatchIntakeConstants.RotaryPiston.CONTRACT_TIME, HatchIntakeConstants.LinearPiston.OUT_TIME),
            STEP_THREE = HatchIntakeConstants.RotaryPiston.EXPAND_TIME + STEP_TWO,
            STEP_FOUR = HatchIntakeConstants.LinearPiston.IN_TIME + STEP_THREE,
            STEP_FIVE = MOVE_BACK_TIME + STEP_FOUR;
    }

    public static class ScoreBallHighTimes {
        public static final long
            STEP_TWO = BallIntakeConstants.LinearLockPiston.CLOSE_TIME,
            STEP_THREE = BallIntakeConstants.RotaryPiston.OPEN_TIME + STEP_TWO;
    }

    public static class ScoreBallLowTimes {
        public static final long
            STEP_TWO = BallIntakeConstants.LinearRetainPiston.CLOSE_TIME + BALL_MOVE_OUT_TIME;
    }

    public static class LoadBallTimes {
        public static final long
            STEP_TWO = BallIntakeConstants.LinearLockPiston.CLOSE_TIME;
    }

}
