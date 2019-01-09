package autonomous.commands;

import frc.robot.Robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.driving.DriveTrain;
import robotcode.driving.DriveTrain.LinearVelocity;
import robotcode.driving.DriveTrain.RotationalVelocity;
/**
 * 
 * @author 3419
 *	Auto command that moves the robot in a parameterized path of form (acos(bt+c), asin(bt+c))
 */
public class ParameterizedPathDriveCommand extends BaseAutonomousCommand {
	
	private Robot mRobot;
	
	private double mCosA;
	private double mCosB;
	private double mCosC;
	private double mSinA;
	private double mSinB;
	private double mSinC;
	
	private double mTotalTime;

	public ParameterizedPathDriveCommand(Robot pRobot, double pCosA, double pCosB, double pCosC, double pSinA,
			double pSinB, double pSinC, double pTotalTime) 
	{
		mRobot = pRobot;
		mCosA = pCosA;
		mCosB = pCosB;
		mCosC = pCosC;
		mSinA = pSinA;
		mSinB = pSinB;
		mSinC = pSinC;
		mTotalTime = pTotalTime;
	}
	
	@Override
	public boolean runCommand() 
	{
		double currentTime = getMillisecondsSinceStart();
		
		double xComp = ResourceFunctions.cosineDerivative(mCosA, mCosB, mCosC, currentTime);
		double yComp = ResourceFunctions.sineDerivative(mSinA, mSinB, mSinC, currentTime);
		
		Vector linearV = new Vector(xComp, yComp);
		
		SmartDashboard.putNumber("X comp", xComp);
		SmartDashboard.putNumber("y comp", yComp);
		SmartDashboard.putNumber("Path angle", -linearV.getAngle() + 90);
		SmartDashboard.putNumber("Path magnitude", linearV.getMagnitude()*100);
		
		DriveTrain mDriveTrain = mRobot.getDriveTrain();
		mDriveTrain.enactMovement(0, 
								  -linearV.getAngle() + 90, 
								  LinearVelocity.NORMAL, 
								  linearV.getMagnitude()*100, 
								  RotationalVelocity.NONE);
		
		return currentTime > mTotalTime;
	}

}
