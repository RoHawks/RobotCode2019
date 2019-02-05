package constants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Ports {

	//*******************//
	// GENERAL VARIABLES //
	//*******************//
	public static final SerialPort.Port NAVX = Port.kUSB;

	public static final int
		//Controllers
		XBOX = 0,
		JOYSTICK = 1,
	
		COMPRESSOR = 0;
	
	//************************//
	// ACTUAL ROBOT VARIABLES //
	//************************//
	public static class ActualRobot {	

		// DRIVETRAIN
		public static final int[] 
			TURN = new int[] { 1, 2, 3, 0 },
			DRIVE = new int[] { 10, 9, 7, 6 };// Right back, right front, left front, left back
		

		// HATCH INTAKE
		public static final int
			HATCH_ROTARY_SOLENOID_IN = 1,
			HATCH_ROTARY_SOLENOID_OUT = 7,
			HATCH_LINEAR_SOLENOID_IN = 5,
			HATCH_LINEAR_SOLENOID_OUT = 6;

		// LEADSCREW
		public static final int
			LEADSCREW = 11;

		// BALL MOTOR INTAKE TZ get rid
		public static final int	
			BALL_HOLDER = 8;

		// BALL PISTON INTAKE
		public static final int
			BALL_ROTARY = 0,
			BALL_LOCK = 0,
			BALL_RETAIN = 0;

		// CLIMBER
		public static final int
			CLIMB_FRONT = 0,
			CLIMB_BACK = 0,
			CLIMB_DRIVE = 0,

			SHIFTER_SOLENOID_IN = 0,
			SHIFTER_SOLENOID_OUT = 0;

	}
	
	//***************************//
	// PROTOTYPE ROBOT VARIABLES //
	//***************************//
	public static class PrototypeRobot {
		public static final int[] 
			TURN = new int[] { 0, 2, 4, 6 },
			DRIVE = new int[] { 1, 3, 5, 7 }; //SW, SE, NE, NW
	}
}
