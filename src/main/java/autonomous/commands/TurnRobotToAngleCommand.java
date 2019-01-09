package autonomous.commands;

import frc.robot.Robot;

import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;

/**
 * 
 * @author 3419
 *	Auto command that turns the *robot* to the desired angle (mAngle).
 */
public class TurnRobotToAngleCommand extends BaseAutonomousCommand {
	private Robot mRobot;
	private double mAngle;

	public TurnRobotToAngleCommand(Robot pRobot, double pAngle) 
	{
		mRobot = pRobot;
		mAngle = pAngle;
	}

	@Override
	public boolean runCommand() 
	{
		DriveTrain mDriveTrain = mRobot.getDriveTrain();
		
		mDriveTrain.enactMovement(mAngle, 
								  mDriveTrain.getDesiredRobotVel().getAngle(), 
								  LinearVelocity.NONE,
								  0.0, 
								  RotationalVelocity.POV);
		
		return mDriveTrain.gyroInRange();
	}

}
