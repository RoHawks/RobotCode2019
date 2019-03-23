package constants;

public class RunConstants {

	public static final boolean
		RUNNING_DRIVE = true,
		RUNNING_PNEUMATICS = true,
		RUNNING_LEADSCREW = true,
		RUNNING_HATCH = RUNNING_PNEUMATICS && true,
		RUNNING_BALL = RUNNING_PNEUMATICS && true,
		RUNNING_CLIMBER = true,
		RUNNING_CAMERA = true,
	
		
		SECONDARY_JOYSTICK = true, // using the box instead of logitech attack
		IS_PROTOTYPE = false,
		LOGGING = false,
		
		RUNNING_EVERYTHING = true && RUNNING_DRIVE && RUNNING_PNEUMATICS && RUNNING_LEADSCREW && RUNNING_BALL && RUNNING_HATCH && SECONDARY_JOYSTICK && RUNNING_CLIMBER;
}