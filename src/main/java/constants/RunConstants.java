package constants;

public class RunConstants {
	public static boolean
		RUNNING_DRIVE = false,
		RUNNING_HATCH = true,
		RUNNING_PNEUMATICS = false,
		RUNNING_CAMERA = false,
		SECONDARY_JOYSTICK = false,
		IS_PROTOTYPE = false,
		
		RUNNING_EVERYTHING = RUNNING_DRIVE && RUNNING_PNEUMATICS && RUNNING_HATCH && SECONDARY_JOYSTICK;
}