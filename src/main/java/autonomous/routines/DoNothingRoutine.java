package autonomous.routines;

import java.util.ArrayList;

import autonomous.AutonomousRoutine;
import autonomous.commands.AutonomousCommand;

public class DoNothingRoutine implements AutonomousRoutine
{
	public ArrayList<AutonomousCommand> getAutonomousCommands()
	{
		return new ArrayList<AutonomousCommand>();	
	}
}
