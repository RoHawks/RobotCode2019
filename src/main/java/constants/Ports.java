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
			LEADSCREW = 11,
			HATCH_ROTARY_SOLENOID_IN = 1,
			HATCH_ROTARY_SOLENOID_OUT = 7,
			HATCH_LINEAR_SOLENOID_IN = 5,
			HATCH_LINEAR_SOLENOID_OUT = 6;

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
