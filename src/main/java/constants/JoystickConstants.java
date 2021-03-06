package constants;

public class JoystickConstants {

    public static final int BUTTONS = 25;

    public static final int
        NUM_PROFILES = 9,
        FULL_PROFILE_1 = 0,
        FULL_PROFILE_2 = 1,
        LEADSCREW_PROFILE = 2,
        HATCH_PROFILE = 3,
        BALL_PROFILE = 4,
        LEADSCREW_HATCH_PROFILE = 5,
        LEADSCREW_BALL_PROFILE = 6,
        CLIMB_PROFILE = 7,
        NONE = 8;

    public static final long
        MILLISECONDS_RESET = 1000;
        
        // NUM_PROFILES = 4,
        // HATCH_PROFILE = 0,
        // LEADSCREW_PROFILE = 0,
        // BALL_PROFILE = 1,
        // FULL_PROFILE = 2,
        // CLIMB_PROFILE = 3;

    public static class LeadscrewButtons {
        public static final int
            CAMERA_ALIGN = 2 + (LEADSCREW_PROFILE * 10),
            PID_TEST = 3 + (LEADSCREW_PROFILE * 10),
            INCREASE_GOAL = 4 + (LEADSCREW_PROFILE * 10), //DC change this potentially
            DECREASE_GOAL = 5 + (LEADSCREW_PROFILE * 10),
            MANUAL = 6 + (LEADSCREW_PROFILE * 10);
        
            // MANUAL = 6,
            // CENTER = 3,
            // CAMERA_ALIGN = 2;
    }

    public static class HatchIntakeButtons {
        public static final int 
            EXPAND = 6 + (HATCH_PROFILE * 10),
            CONTRACT = 7 + (HATCH_PROFILE * 10),
            EXPAND_CONTRACT_OPPOSITE = 4 + (HATCH_PROFILE * 10),
            IN = 10 + (HATCH_PROFILE * 10),
            OUT = 11 + (HATCH_PROFILE * 10),
            IN_OUT_OPPOSITE = 5 + (HATCH_PROFILE * 10);
    }

    public static class IntakeButtons {
        public static final int 
            INTAKE_HATCH = 6 + (LEADSCREW_HATCH_PROFILE * 10),
            SCORE_HATCH = 7 + (LEADSCREW_HATCH_PROFILE * 10),
            INTAKE_BALL = 8 + (LEADSCREW_HATCH_PROFILE * 10),
            SCORE_BALL_HIGH = 9 + (LEADSCREW_HATCH_PROFILE * 10),
            SCORE_BALL_LOW = 10 + (LEADSCREW_HATCH_PROFILE * 10),
            MANUAL = 11 + (LEADSCREW_HATCH_PROFILE * 10);
    }


    public static class BallGateButtons {
        public static final int
            RETAIN = 2 + (BALL_PROFILE * 10),
            UNRETAIN = 3 + (BALL_PROFILE * 10),
            LOCK = 4 + (BALL_PROFILE * 10),
            UNLOCK = 5 + (BALL_PROFILE * 10),
            SCORE = 6 + (BALL_PROFILE * 10),
            UNSCORE = 7 + (BALL_PROFILE * 10);
            // MANUAL = 2 + (BALL_PROFILE * 10),
            // HOLD = 3 + (BALL_PROFILE * 10),
            // SCORE_ROCKET = 4 + (BALL_PROFILE * 10),
            // SCORE_CARGOSHIP = 5 + (BALL_PROFILE * 10);
    }

    public static class ClimbButtons{
        public static final int
            SHIFT = 2 + (CLIMB_PROFILE * 10),
            DRIVE = 3 + (CLIMB_PROFILE * 10),
            BACK = 4 + (CLIMB_PROFILE * 10),
            FRONT = 5 + (CLIMB_PROFILE * 10);
    }

    public static class FinalRobotButtons {
        public static final int
            SCORE_PANEL_CARGO = 5,
            LOAD_PANEL = 7,
            LEADSCREW_OVERRIDE = 3,
            SCORE_PANEL_ROCKET = 4,
            HAS_LOADED_PANEL = 6,
            HAS_SCORED_PANEL = 1,
            LEFT_ROCKET_TOP = 15,
            LEFT_ROCKET_BOTTOM = 16,
            RIGHT_ROCKET_TOP = 2,
            RIGHT_ROCKET_BOTTOM = 14,
            SCORE_BALL_ROCKET = 22,
            SCORE_BALL_CARGO = 23,
            LOAD_BALL = 25,
            HAS_LOADED_BALL = 24,
            HAS_SCORED_BALL = 21,
            BALL_PANEL_SWITCH = 8, //when ball, off, when panel, on
            CLIMB = 12,
            CLIMB_ABORT = 13,
            CLIMB_AUTO_MANUAL_SWITCH = 11; //when on, Auto, when off, Manual
    }
}