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
			TURN = new int[] { 6, 1, 3, 11 },
			DRIVE = new int[] { 8, 7, 5, 13 }; // Right back, right front, left front, left back

		// HATCH INTAKE
		public static final int
			LEADSCREW = 11,
			HATCH_ROTARY_SOLENOID = 0,
			HATCH_LINEAR_SOLENOID = 0;

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
