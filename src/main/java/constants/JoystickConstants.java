package constants;

public class JoystickConstants {

    public static final int
        HATCH_PROFILE = 0,
        LEADSCREW_PROFILE = 0,
        BALL_PROFILE = 1;
    
    public static class HatchIntakeButtons {
        public static final int 
            EXPAND_CONTRACT = 4,
            IN_OUT = 5;
    }

    public static class LeadscrewButtons {
        public static final int
            MANUAL = 1,
            CAMERA_ALIGN = 2,
            LOADING_STATION = 3;
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
}
