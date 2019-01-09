package autonomous;

import java.util.ArrayList;

import autonomous.commands.AutonomousCommand;

public interface AutonomousRoutine {
	
	ArrayList<AutonomousCommand> getAutonomousCommands();
}
