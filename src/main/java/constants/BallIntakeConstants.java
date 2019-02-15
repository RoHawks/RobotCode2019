/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * Add your docs here.
 */
public class BallIntakeConstants {

    // *******************************//
    // MOTOR INITIALIZATION CONSTANTS //
    // *******************************//
    public static final boolean 
        ENCODER_REVERSED = true,
        REVERSED = false;

    public static final int 
        LOWEST_POINT = -1644,
        HIGHEST_POINT = -2865;


    // **************//
    // PID CONSTANTS //
    // **************//
    public static class PID {

        public static final double 
            HOLDER_P = 0.1, 
            HOLDER_I = 0.0007, // 0.0003
            HOLDER_D = 0.0001;

        public static final int 
            HOLDER_TOLERANCE = 500, 
            HOLDER_IZONE = 1000;
    }


    // *******************//
    // MOVEMENT POSITIONS //
    // *******************//
    public static class Positions {
        public static final int 
            HOLDER_HOLD = 0, 
            HOLDER_SCORE_ROCKET_POSITION = 0, 
            HOLDER_SCORE_CARGOSHIP_POSITION = 0;
    }

    public static class RotaryPiston {
        public static final Value 
            FORWARD = Value.kForward,
            BACKWARD = Value.kReverse;

        public static final long
            OPEN_TIME = 500,
            CLOSE_TIME = 500;
    }

    public static class LinearLockPiston {
        public static final Value 
            LOCK = Value.kForward,
            LET_GO = Value.kReverse;

        public static final long
            OPEN_TIME = 500,
            CLOSE_TIME = 500;
    }

    public static class LinearRetainPiston {
        public static final Value 
            RETAIN = Value.kForward,
            RELEASE = Value.kReverse;

        public static final long
            OPEN_TIME = 500,
            CLOSE_TIME = 500;
    }
}
