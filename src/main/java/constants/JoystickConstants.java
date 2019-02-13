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
            HAS_SCORED = 6;//27;

    }
}


// package constants;

// public class JoystickConstants {

//     public static final int
//         NUM_PROFILES = 9,
//         FULL_PROFILE_1 = 0,
//         FULL_PROFILE_2 = 1,
//         LEADSCREW_PROFILE = 2,
//         HATCH_PROFILE = 3,
//         BALL_PROFILE = 4,
//         LEADSCREW_HATCH_PROFILE = 5,
//         LEADSCREW_BALL_PROFILE = 6,
//         CLIMB_PROFILE = 7,
//         NONE = 8;
        
//         // NUM_PROFILES = 4,
//         // HATCH_PROFILE = 0,
//         // LEADSCREW_PROFILE = 0,
//         // BALL_PROFILE = 1,
//         // FULL_PROFILE = 2,
//         // CLIMB_PROFILE = 3;

//     public static class LeadscrewButtons {
//         public static final int
//             CAMERA_ALIGN = 2 + (LEADSCREW_PROFILE * 10),
//             PID_TEST = 3 + (LEADSCREW_PROFILE * 10),
//             INCREASE_GOAL = 4 + (LEADSCREW_PROFILE * 10), //DC change this potentially
//             DECREASE_GOAL = 5 + (LEADSCREW_PROFILE * 10),
//             MANUAL = 6 + (LEADSCREW_PROFILE * 10);
        
//             // MANUAL = 6,
//             // CENTER = 3,
//             // CAMERA_ALIGN = 2;
//     }

//     public static class HatchIntakeButtons {
//         public static final int 
//             EXPAND = 6 + (HATCH_PROFILE * 10),
//             CONTRACT = 7 + (HATCH_PROFILE * 10),
//             EXPAND_CONTRACT_OPPOSITE = 4 + (HATCH_PROFILE * 10),
//             IN = 10 + (HATCH_PROFILE * 10),
//             OUT = 11 + (HATCH_PROFILE * 10),
//             IN_OUT_OPPOSITE = 5 + (HATCH_PROFILE * 10);
//     }

//     public static class IntakeButtons {
//         public static final int 
//             INTAKE_HATCH = 7 + (LEADSCREW_HATCH_PROFILE * 10),
//             SCORE_HATCH = 8 + (LEADSCREW_HATCH_PROFILE * 10),
//             INTAKE_BALL = 9 + (LEADSCREW_HATCH_PROFILE * 10),
//             SCORE_BALL = 10 + (LEADSCREW_HATCH_PROFILE * 10),
//             MANUAL = 11 + (LEADSCREW_HATCH_PROFILE * 10);
//     }


//     public static class BallGateButtons {
//         public static final int
//             SCORE_ROCKET = 4 + (BALL_PROFILE * 10),
//             SCORE_CARGOSHIP = 5 + (BALL_PROFILE * 10),
//             HOLD = 3 + (BALL_PROFILE * 10),
//             MANUAL = 2 + (BALL_PROFILE * 10);
//     }

//     public static class ClimbButtons{
//         public static final int
//             SHIFT = 2 + (CLIMB_PROFILE * 10),
//             FRONT = 5 + (CLIMB_PROFILE * 10),
//             BACK = 4 + (CLIMB_PROFILE * 10),
//             DRIVE = 3 + (CLIMB_PROFILE * 10);
//     }

//     public static class FinalRobotButtons {
//         public static final int
//             SCORE = 16,
//             LOAD = 18,
//             LEADSCREW_OVERRIDE = 0,
//             SCORE_BACKWARDS = 0,
//             HAS_LOADED = 17,
//             HAS_SCORED = 6;

//     }
// }