package autonomous.commands;

import frc.robot.Robot;

import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;

/**
 * 
 * @author 3419
 *	Auto command that makes the Robot drive in a straight line. Robot drives towards angle (mAngle)
 *  for (mTotalMilliseconds) seconds, with velocity moving from mInitialVelocity to mFinalVelocity.
 */
public class StraightLineDriveCommand extends BaseAutonomousCommand {

	private Robot mRobot;
	
	private double mAngle;
	private double mInitialVelocity;
	private double mFinalVelocity;
	private long mTotalMilliseconds;

	public StraightLineDriveCommand(Robot pRobot, double pAngle, double pInitialVelocity, double pFinalVelocity, 
			long pTotalMilliseconds) 
	{
		mRobot = pRobot;
		mAngle = pAngle;
		mInitialVelocity = pInitialVelocity;
		mFinalVelocity = pFinalVelocity;
		mTotalMilliseconds = pTotalMilliseconds;
	}

	@Override
	public boolean runCommand() 
	{
		// compute what percent into the command we are:
		double percentComplete = ((float)getMillisecondsSinceStart()) / mTotalMilliseconds;
		
		// calculate speed for the robot:
		double totalSpeedRange = mFinalVelocity - mInitialVelocity; // positive in acceleration, negative in deceleration, 0
																	// in constant
		double portionIntoSpeedRange = totalSpeedRange * percentComplete;
		double intialCalculatedspeed = mInitialVelocity + portionIntoSpeedRange;

		// check if we should finish this command:
		// TODO this is set to a default time, but can be if some action has past instead...
		boolean isThisCommandDone = getMillisecondsSinceStart() > mTotalMilliseconds;
		
		// if command is going to wrap up after this run, set speed to final velocity:
		double finalSpeed = isThisCommandDone ? mFinalVelocity : intialCalculatedspeed;
		
		// drive the robot where it needs to go:
		DriveTrain mDriveTrain = mRobot.getDriveTrain();
		mDriveTrain.enactMovement(mDriveTrain.getRobotAngle(), 
								  mAngle, 
								  LinearVelocity.NORMAL, 
								  finalSpeed,
								  RotationalVelocity.NONE);

		return isThisCommandDone;

	}

}
