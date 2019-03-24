package autonomous.routines;

import java.util.ArrayList;

import frc.robot.Robot;

import autonomous.AutonomousRoutine;
import autonomous.commands.AutonomousCommand;
import autonomous.commands.ParameterizedPathDriveCommand;
import autonomous.commands.StopCommand;
import autonomous.commands.StraightLineDriveCommand;
import autonomous.commands.TurnRobotToAngleCommand;
import constants.AutoConstants;

public class DefaultRoutine implements AutonomousRoutine {

	private Robot mRobot;

	public DefaultRoutine(Robot pRobot) {
		mRobot = pRobot;
	}

	@Override
	public ArrayList<AutonomousCommand> getAutonomousCommands() {
		ArrayList<AutonomousCommand> returnValue = new ArrayList<AutonomousCommand>();

		// accelerate
		returnValue.add(new StraightLineDriveCommand(
				mRobot,
				AutoConstants.DefaultRoutine.WHEEL_ANGLE,
				AutoConstants.DefaultRoutine.MINIMUM_SPEED,
				AutoConstants.DefaultRoutine.MAXIMUM_SPEED,
				AutoConstants.DefaultRoutine.ACCELERATION_TIME));

		// drive full speed
		returnValue.add(new StraightLineDriveCommand(
				mRobot,
				AutoConstants.DefaultRoutine.WHEEL_ANGLE,
				AutoConstants.DefaultRoutine.MAXIMUM_SPEED,
				AutoConstants.DefaultRoutine.MAXIMUM_SPEED,
				AutoConstants.DefaultRoutine.DRIVE_FULL_SPEED_TIME));

		// slow down then stop
		returnValue.add(new StraightLineDriveCommand(
				mRobot,
				AutoConstants.DefaultRoutine.WHEEL_ANGLE,
				AutoConstants.DefaultRoutine.MAXIMUM_SPEED,
				AutoConstants.DefaultRoutine.MINIMUM_SPEED,
				AutoConstants.DefaultRoutine.DECELERATION_TIME));

		returnValue.add(new StopCommand(mRobot));
		returnValue.add(new TurnRobotToAngleCommand(mRobot, -90)); // TODO this is a default angle setting
		returnValue.add(new StopCommand(mRobot));
		
		return returnValue;
	}

}
