package constants;

public class RunConstants {
	public static boolean
		RUNNING_DRIVE = true,
		RUNNING_PNEUMATICS = true,
		RUNNING_LEADSCREW = true,
		RUNNING_HATCH = RUNNING_PNEUMATICS && true,
		RUNNING_BALL = RUNNING_PNEUMATICS && true,
		RUNNING_CLIMBER = false,
		RUNNING_CAMERA = true,
		SECONDARY_JOYSTICK = true, // using the box instead of logitech attack
		IS_PROTOTYPE = false,
		
		RUNNING_EVERYTHING = true && RUNNING_DRIVE && RUNNING_PNEUMATICS && RUNNING_LEADSCREW && RUNNING_HATCH && SECONDARY_JOYSTICK;
}