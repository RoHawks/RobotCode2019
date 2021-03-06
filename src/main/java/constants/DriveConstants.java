
package constants;

public class DriveConstants {

	//*******************//
	// GENERAL VARIABLES //
	//*******************//
	public static final double  // speed mins, when lower than these don't do anything
		MIN_LINEAR_VELOCITY = 0.02, // determines NORMAL vs ANGLE_ONLY linear velocity mode
		MIN_DIRECTION_MAG = 0.25, // refers to joystick magnitudes
		MAX_INDIVIDUAL_VELOCITY = 1.0; // max length of wheel vectors
		
	public static final double[]
		INDIVIDUAL_SCALE_FACTORS = { 1.0 , 1.0 , 1.0 , 1.0 };
	
	public static final double
		EMERGENCY_VOLTAGE = 10000,
		MAX_EMERGENCY_VOLTAGE = 0.5;

	public static final double 
		MAX_ANGULAR_VELOCITY = 0.9, // does absolutely nothing lol
		MAX_LINEAR_VELOCITY = 0.9; // absolute max linear velocity (no scaling)

	
	//*****************//
	// SWERVE SPEEDS //
	//*****************//
	public static class SwerveSpeeds {
		public static final double 
			SPEED_MULT = 0.75,
			ANGULAR_SPEED_MULT = 0.75,
			NUDGE_MOVE_SPEED = 0.2,
			NUDGE_TURN_SPEED = 0.15;
	}

	
	//**************************//
	// ACTUAL ROBOT VARIABLES //
	//**************************//
	public static class ActualRobot {
		public static final boolean[]
			TURN_INVERTED = new boolean[] { false, false, true, true },
			DRIVE_INVERTED = new boolean[] { false, false, false, false },
			ENCODER_REVERSED = new boolean[] { false, false, true, true };
	
		public static final double[] 
			X_OFF = new double[] { -21.5/2.0, 21.5/2.0 , 21.5/2.0 , -21.5/2.0 }, 
			Y_OFF = new double[] { 21.938/2.0, 21.938/2.0 , -21.938/2.0 , -29.138/2.0 }, 
			
			ROTATION_P = new double[] { 1.0, 1.0, 1.0, 1.0 },
			ROTATION_I = new double[] { 0.001, 0.001, 0.001, 0.001 },
			ROTATION_D = new double[] { 0, 0, 0, 0 };

		public static final int[] 
			OFFSETS = new int[] { 2311, 125 , 899, 528 },
			
			ROTATION_IZONE = new int[] { 500, 500, 500, 500 },
			ROTATION_TOLERANCE = new int[] { 5, 5, 5, 5 };
			
		public static final double
			GYRO_P = 0.004,
			GYRO_I = 0.00002,
			GYRO_D = 0,
			GYRO_TOLERANCE = 5,
			GYRO_MAX_SPEED = 1,
			
			DRIFT_COMP_P = 0.03, //0.06
			DRIFT_COMP_I = 0, //0.0006
			DRIFT_COMP_D = 0,
			DRIFT_COMP_MAX = 0.3;
	}
	
	
	//***************************//
	// PROTOTYPE ROBOT VARIABLES //
	//***************************//
	public static class PrototypeRobot { //SW, SE, NE, NW
		public static final boolean[]
			TURN_INVERTED = new boolean[] { true, true, true, true },
			DRIVE_INVERTED = new boolean[] { false, false, false, false },
			ENCODER_REVERSED = new boolean[] { true, true, true, true };
		
		public static final double[] 
			X_OFF = new double[] { -19.0 / 2.0, -19.0 / 2.0, 19.0 / 2.0, 19.0 / 2.0 }, //27.5 along y, 29.25 along x
			Y_OFF = new double[] { -22.0 / 2.0, 22.0 / 2.0, 22.0 / 2.0, -22.0 / 2.0 },
			
			ROTATION_P = new double[] { 0.7, 0.7, 0.7, 0.7 },
			ROTATION_I = new double[] { 0.007, 0.007, 0.007, 0.007 },
			ROTATION_D = new double[] { 0, 0, 0, 0 };

		public static final int[] 
			OFFSETS = new int[] { 3669, 3228, 1130, 2256 },
			ROTATION_IZONE = new int[] { 500, 500, 500, 500 },
			ROTATION_TOLERANCE = new int[] { 3, 3, 3, 3 };
		
		public static final double 
			GYRO_P = 0.00085,
			GYRO_I = 0.0003,
			GYRO_D = 0,
			GYRO_TOLERANCE = 5,
			GYRO_MAX_SPEED = 1,

			DRIFT_COMP_P = 0.08,
			DRIFT_COMP_I = 0.0008,
			DRIFT_COMP_D = 0,
			DRIFT_COMP_MAX = 0.3;
	}

}