package constants;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.SerialPort.Port;

public class Ports {

	//*******************//
	// GENERAL VARIABLES //
	//*******************//
	public static final SerialPort.Port NAVX = Port.kMXP;

	public static final int
		//Controllers
		XBOX = 0,
		JOYSTICK = 1,
		CLIMB_JOYSTICK = 2,
	
		COMPRESSOR = 0;
	
	//************************//
	// ACTUAL ROBOT VARIABLES //
	//************************//
	public static class ActualRobot {	

		// DRIVETRAIN
		public static final int[] 
			TURN = new int[] { 11, 1, 2, 10 },
			DRIVE = new int[] { 3, 4, 1, 2 }; // Right back, right front, left front, left back
		
		// HATCH INTAKE
		public static final int
			HATCH_ROTARY_SOLENOID_IN = 5,
			HATCH_LINEAR_SOLENOID_IN = 1;

		// LEADSCREW
		public static final int
			LEADSCREW = 4;

		// BALL PISTON INTAKE
		public static final int
			BALL_ROTARY = 2,
			BALL_LOCK = 0,
			BALL_RETAIN = 4;

		public static final	int
			ULTRASONIC_SENSOR_RIGHT = 0,
			ULTRASONIC_SENSOR_LEFT = 1;

		// CLIMBER
		public static final int
			CLIMB_DRIVE0 = 5,
			CLIMB_DRIVE1 = 8,
			CLIMB_FRONT_PISTON_IN = 2,
			CLIMB_FRONT_PISTON_OUT  = 3,
			CLIMB_BACK_PISTON_IN = 0,
			CLIMB_BACK_PISTON_OUT = 1,
			CLIMB_FRONT_PISTON_BREAK = 4,
			CLIMB_FRONT_PISTON_THROUGH = 5,
			CLIMB_BACK_PISTON_BREAK = 6,
			CLIMB_BACK_PISTON_THROUGH = 7,

			CLIMB_FRONT_RELAY = 0,
			CLIMB_BACK_RELAY_BRAKE = 2,
			CLIMB_BACK_RELAY_THROUGH = 1;

		// LEDs
		public static final int
			BLINKIN_CHANNEL = 0;

	}
	
	//***************************//
	// PROTOTYPE ROBOT VARIABLES //
	//***************************//
	public static class PrototypeRobot {

		// DRIVETRAIN
		public static final int[] 
			TURN = new int[] { 1, 2, 3, 0 },
			DRIVE = new int[] { 10, 9, 7, 6 }; // Right back, right front, left front, left back
		

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
			CLIMB_FRONT = 6,
			CLIMB_BACK = 7,
			CLIMB_DRIVE = 8,

			SHIFTER_SOLENOID_IN = 0,
			SHIFTER_SOLENOID_OUT = 0;
	}
}
