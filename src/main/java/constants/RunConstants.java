package constants;

public class RunConstants {
	public static boolean
		RUNNING_DRIVE = true,
		RUNNING_PNEUMATICS = true,
		RUNNING_CAMERA = false,
		SECONDARY_JOYSTICK = true,
		IS_PROTOTYPE = true,
		
		RUNNING_EVERYTHING = RUNNING_DRIVE && RUNNING_PNEUMATICS && SECONDARY_JOYSTICK;
}