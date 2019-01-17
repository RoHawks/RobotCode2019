package constants;

public class RunConstants {
	public static boolean
		RUNNING_DRIVE = true,
		RUNNING_HATCH = true,
		RUNNING_LEADSCREW = true,
		RUNNING_PNEUMATICS = true,
		RUNNING_CAMERA = true,
		SECONDARY_JOYSTICK = true,
		IS_PROTOTYPE = false,
		
		RUNNING_EVERYTHING = RUNNING_DRIVE && RUNNING_PNEUMATICS && RUNNING_LEADSCREW && RUNNING_HATCH && SECONDARY_JOYSTICK;
}