package autonomous.commands;

public abstract class BaseAutonomousCommand implements AutonomousCommand {
	protected long mTimeStartMillis;

	public void startup() 
	{
		mTimeStartMillis = System.currentTimeMillis();
	}

	protected long getMillisecondsSinceStart() 
	{
		return System.currentTimeMillis() - mTimeStartMillis;
	}

}
