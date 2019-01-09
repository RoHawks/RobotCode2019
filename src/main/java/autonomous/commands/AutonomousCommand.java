package autonomous.commands;

public interface AutonomousCommand {
	
	void startup();
	boolean runCommand();//true if complete, false if still running
}
