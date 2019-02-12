package constants;

public class JoystickConstants {

    public static final int
        HATCH_PROFILE = 0,
        LEADSCREW_PROFILE = 0,
        BALL_PROFILE = 1,
        FULL_PROFILE = 2,
        CLIMB_PROFILE = 3;
    
    public static class HatchIntakeButtons {
        public static final int 
            EXPAND_CONTRACT = 4,
            IN_OUT = 5;
    }

    public static class LeadscrewButtons {
        public static final int
            MANUAL = 6,
            CAMERA_ALIGN = 2;
    }

    public static class IntakeButtons {
        public static final int 
            INTAKE_HATCH = 7,
            SCORE_HATCH = 8,
            INTAKE_BALL = 9,
            SCORE_BALL = 10,
            MANUAL = 11;
    }


    public static class BallGateButtons {
        public static final int
            SCORE_ROCKET = 14, //4
            SCORE_CARGOSHIP = 15, //5
            HOLD = 13, //3
            MANUAL = 12; //2
    }

    public static class ClimbButtons{
        public static final int
            SHIFT = 32,
            FRONT = 35,
            BACK = 34,
            DRIVE = 33;
    }

    public static class FinalRobotButtons {
        public static final int
            SCORE = 22,
            LOAD = 23,
            LEADSCREW_OVERRIDE = 24,
            SCORE_BACKWARDS = 25,
            HAS_LOADED = 26,
            HAS_SCORED = 27;

    }
}
