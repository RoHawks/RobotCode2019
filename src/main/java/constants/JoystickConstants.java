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
            SCORE = 16,//22,
            LOAD = 18,//23,
            LEADSCREW_OVERRIDE = 24,
            SCORE_BACKWARDS = 25,
            HAS_LOADED = 17,//26,
            HAS_SCORED = 6,//27;
            LOADING_STATION = 11,
            CARGO_SHIP_FRONT = 7,
            LEFT_ROCKET_SIDE = 5,
            RIGHT_ROCKET_SIDE = 9,
            LEFT_ROCKET_TOP = 13,
            LEFT_ROCKET_BOTTOM = 12,
            RIGHT_ROCKET_TOP = 8,
            RIGHT_ROCKET_BOTTOM = 10,
            CARGO_SHIP_LEFT = 9,
            CARGO_SHIP_RIGHT = 5;

    }
}
