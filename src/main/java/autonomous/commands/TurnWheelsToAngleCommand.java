package autonomous.commands;

import frc.robot.Robot;

import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;

/**
 * 
 * @author 3419
 *	Auto command that turns the *wheels* to the desired angle (mAngle).
 */
public class TurnWheelsToAngleCommand extends BaseAutonomousCommand {
	private Robot mRobot;
	private double mAngle;

	public TurnWheelsToAngleCommand(Robot pRobot, double pAngle) 
	{
		mRobot = pRobot;
		mAngle = pAngle;
	}

	@Override
	public boolean runCommand() 
	{
		DriveTrain mDriveTrain = mRobot.getDriveTrain();
		
		mDriveTrain.enactMovement(mDriveTrain.getRobotAngle(), 
								  mAngle, 
								  LinearVelocity.ANGLE_ONLY,
								  0, 
								  RotationalVelocity.NONE);
		
		return mDriveTrain.allWheelsInRange(mAngle);
	}

}
