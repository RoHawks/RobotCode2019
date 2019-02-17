package robotcode.driving;

import constants.DriveConstants;
import constants.JoystickConstants;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import resource.Vector;
import robotcode.LocalJoystick;
import robotcode.pid.GenericPIDOutput;
import robotcode.pid.LocalPIDController;
import sensors.RobotAngle;

public class DriveTrain {

	// Output
	private SwerveDrive mSwerveDrive;
	private Wheel[] mWheels;

	// Input
	private RobotAngle mRobotAngle;
	private XboxController mController;
	private double mJoystickAngle;
	private LocalJoystick mJoystick;

	// End products
	private Vector mDesiredRobotVel;
	private double mDesiredAngularVel;

	private boolean mIsFieldRelative;

	// PIDs
	private LocalPIDController mGyroPID;
	private GenericPIDOutput mGyroOutput;

	private LocalPIDController mDriftCompensationPID;
	private GenericPIDOutput mDriftCompensationOutput;

	// Velocity modes
	private LinearVelocity mLinearVel;
	private LinearVelocity mPrevLinearVel;
	private RotationalVelocity mRotationalVel;

	public DriveTrain(Wheel[] pWheels, XboxController pController, RobotAngle pRobotAngle, LocalJoystick pJoystick) {
		mWheels = pWheels;
		mController = pController;
		mSwerveDrive = new SwerveDrive(mWheels);
		mJoystick = pJoystick;

		mRobotAngle = pRobotAngle;

		mDesiredRobotVel = new Vector();
		mDesiredAngularVel = 0;

		mJoystickAngle = 0;
		mIsFieldRelative = false;

		mLinearVel = LinearVelocity.NONE;
		mPrevLinearVel = LinearVelocity.NONE;
		mRotationalVel = RotationalVelocity.NONE;

		pidInit();
	}

	public enum LinearVelocity {
		ANGLE_ONLY,
		NONE,
		NORMAL,
		NUDGE
	}

	public enum RotationalVelocity {
		NONE,
		NORMAL,
		NUDGE,
		POV,
		SECONDARY
	}

	private void enactMovement() {
		enactMovement(mController.getPOV(), getStickAngle(Hand.kLeft), getLinearVelocityState(), getStickLinearVel(),
				getRotationalVelocityState());
	}

	public void stop(){
		enactMovement(0, 0, LinearVelocity.NONE, 0, RotationalVelocity.NONE);

		// TODO would this be better?
		// for (int i = 0; i < 4; i++) {
		// 	mWheels[i].setLinearVelocity(0);
		// 	mWheels[i].setTurnSpeed(0);
		// }
	}

	public void enactMovement(double pGyroAngle, double pRobotDirectionAngle, LinearVelocity pLinearVel,
			double pSpecificLinearVelocity, RotationalVelocity pRotationalVel) {
		SmartDashboard.putNumber("Robot Angle", mRobotAngle.getAngleDegrees());

		double joystickAngle = pRobotDirectionAngle;
		double robotDirectionAngle = joystickAngle;

		double secondaryJoystickAngle = 0;

		mLinearVel = pLinearVel;
		mRotationalVel = pRotationalVel;

		SmartDashboard.putString("Linear Velocity State", mLinearVel.name());
		SmartDashboard.putString("Previous Linear Velocity", mPrevLinearVel.name());
		SmartDashboard.putString("Rotational Velocity State", mRotationalVel.name());

		if (mController.getStartButtonReleased()) {
			mIsFieldRelative = !mIsFieldRelative;
		}

		if (mIsFieldRelative) {
			robotDirectionAngle = ResourceFunctions.putAngleInRange(joystickAngle - mRobotAngle.getAngleDegrees());
		}

		SmartDashboard.putBoolean("Field Relative", mIsFieldRelative);

		if(mRotationalVel == RotationalVelocity.SECONDARY){
			if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_BOTTOM)){
				secondaryJoystickAngle = 331.3;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_SIDE) || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_RIGHT)){
				secondaryJoystickAngle = 270;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_TOP)){
				secondaryJoystickAngle = 208.7;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_BOTTOM)){
				secondaryJoystickAngle = 28.7;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_SIDE) || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_LEFT)){
				secondaryJoystickAngle = 90;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_TOP)){
				secondaryJoystickAngle = 151.3;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LOADING_STATION)){
				secondaryJoystickAngle = 180;
			}
			else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_FRONT)){
				secondaryJoystickAngle = 0;
			}
		}

		
		Vector linearVel = new Vector();
		switch (mLinearVel) {
			case NORMAL:
				linearVel = Vector.createPolar(robotDirectionAngle, pSpecificLinearVelocity);
				break;
			case NUDGE:
				linearVel = nudgeMove();
				break;
			case ANGLE_ONLY:
				break;
			case NONE:
				break;
			default:
				throw new RuntimeException("Unknown drivetrain linear velocity state");
		}
		mDesiredRobotVel = new Vector(linearVel);
		SmartDashboard.putNumber("Linear velocity after switch", mDesiredRobotVel.getMagnitude());

		switch (mRotationalVel) {
			case NORMAL:
				mGyroPID.reset();
				mDesiredAngularVel = getStickAngularVel();
				break;
			case NUDGE:
				mGyroPID.reset();
				mDesiredAngularVel = nudgeTurn();
				break;
			case NONE:
				mGyroPID.reset();
				mDesiredAngularVel = 0;
				break;
			case POV:
				mDesiredAngularVel = getAngularPIDVel(pGyroAngle); // TZ
				SmartDashboard.putNumber("Gyro PID error", mGyroPID.getError());
				break;
			case SECONDARY:
				mDesiredAngularVel = getAngularPIDVel(secondaryJoystickAngle);
				SmartDashboard.putNumber("Gyro PID error", mGyroPID.getError());
				break;
			default:
				throw new RuntimeException("Unknown drivetrain rotational velocity state");
		}

		SmartDashboard.putNumber("POV", mController.getPOV());
		SmartDashboard.putString("Rotational Velocity State", mRotationalVel.toString());
		if (mRotationalVel == RotationalVelocity.NONE) {
			if (mLinearVel == LinearVelocity.ANGLE_ONLY) {
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(robotDirectionAngle, 0);
				}
				resetDriftCompensation();
				mDriftCompensationPID.setSetpoint(mRobotAngle.getAngleDegrees());
			}
			else if (mLinearVel == LinearVelocity.NONE) {
				if (mPrevLinearVel == LinearVelocity.NUDGE) {
					for (int i = 0; i < 4; i++) {
						mWheels[i].set(mSwerveDrive.getOutput(i).getAngle(), 0);
					}
				}
				else {
					for (int i = 0; i < 4; i++) {
						mWheels[i].setLinearVelocity(0);
						mWheels[i].setTurnSpeed(0);
					}
				}
				resetDriftCompensation();
				mDriftCompensationPID.setSetpoint(mRobotAngle.getAngleDegrees());
			}
			else if (mLinearVel == LinearVelocity.NORMAL || mLinearVel == LinearVelocity.NUDGE) {
				mDriftCompensationPID.enable();
				SmartDashboard.putNumber("Drift comp error", mDriftCompensationPID.getError());
				SmartDashboard.putNumber("Drift comp value", mDriftCompensationOutput.getVal());
				mSwerveDrive.calculateHoldDirection(mDriftCompensationOutput.getVal(), getDesiredRobotVel());
				for (int i = 0; i < 4; i++) {
					mWheels[i].set(mSwerveDrive.getOutput(i));
				}
			}
		}
		else {
			resetDriftCompensation();
			mDriftCompensationPID.setSetpoint(mRobotAngle.getAngleDegrees());
			mSwerveDrive.calculate(getDesiredAngularVel(), getDesiredRobotVel());
			for (int i = 0; i < 4; i++) {
				mWheels[i].set(mSwerveDrive.getOutput(i));
			}
		}

		SmartDashboard.putBoolean("Drift comp enabled", mDriftCompensationPID.isEnabled());

		for (int i = 0; i < 4; i++) {
			SmartDashboard.putNumber("Error " + i + ":", robotDirectionAngle - mWheels[i].getAngle());
			SmartDashboard.putNumber("Angle " + i + ":", mWheels[i].getAngle());
			
		}
	}

	public void driveSwerve() {
		enactMovement();
	}

	/**
	 * tank drive : right trigger = forward ; left trigger = turn ; right joystick x
	 * = turn
	 */
	public void driveTank() {
		double forwardVel = mController.getTriggerAxis(Hand.kRight) - mController.getTriggerAxis(Hand.kLeft);
		forwardVel *= 0.4;
		double angularVel = mController.getX(Hand.kRight);
		angularVel *= 0.1;

		double leftSpeed = forwardVel - angularVel;
		double rightSpeed = forwardVel + angularVel;

		SmartDashboard.putNumber("Tank Left Speed", leftSpeed);
		SmartDashboard.putNumber("Tank Right Speed", rightSpeed);

		mWheels[0].set(0, rightSpeed);
		mWheels[1].set(0, leftSpeed);
		mWheels[2].set(0, leftSpeed);
		mWheels[3].set(0, rightSpeed);
	}

	/**
	 * Crab Drive method
	 */
	public void driveCrab() {
		double linearVelocity = getStickLinearVel();
		double joystickAngle = getStickAngle(Hand.kLeft);
		for (int i = 0; i < 4; i++) {
			mWheels[i].set(joystickAngle, linearVelocity);
			SmartDashboard.putNumber("Angle " + i + ":", mWheels[i].getAngle());
		}
	}

	/**
	 * Gets the angle of Xbox controller joystick
	 * 
	 * @param h
	 *            Joystick to get the angle for
	 * @return Angle of the stick in degrees, with 0 degrees pointing directly up on
	 *         the controller
	 */
	public double getStickAngle(Hand h) {
		double x = mController.getX(h);
		double y = -mController.getY(h);
		SmartDashboard.putNumber("X-Value " + h.toString() + ":", x);
		SmartDashboard.putNumber("Y-Yalue " + h.toString() + ":", y);
		if (Math.abs(y) >= DriveConstants.MIN_DIRECTION_MAG || Math.abs(x) >= DriveConstants.MIN_DIRECTION_MAG) {
			mJoystickAngle = -Math.toDegrees(Math.atan2(y, x)) + 90;
			mJoystickAngle = ResourceFunctions.putAngleInRange(mJoystickAngle); // puts angle between zero and 360
		}
		SmartDashboard.putNumber(h.toString() + " Joystick Angle: ", mJoystickAngle);
		return mJoystickAngle;
	}

	/**
	 * Gets magnitude of Xbox controller joystick
	 * 
	 * @param h
	 *            Joystick to get the magnitude for
	 * @return Magnitude of the depression; 0 is not used, 1 is fully depressed
	 */
	public double getStickMag(Hand h) {
		return Math.hypot(mController.getX(h), mController.getY(h));
	}

	/**
	 * Angular velocity using nudge bumpers
	 * 
	 * @return correct angular velocity
	 */
	private double nudgeTurn() {
		if (mController.getBumper(Hand.kLeft)) {
			return -DriveConstants.SwerveSpeeds.NUDGE_TURN_SPEED;
		}
		else if (mController.getBumper(Hand.kRight)) {
			return DriveConstants.SwerveSpeeds.NUDGE_TURN_SPEED;
		}

		return 0;
	}

	/**
	 * Get the direction vector for nudge driving using the letter buttons
	 * 
	 * @return correct direction vector
	 */
	private Vector nudgeMove() {
		double newAngle = 0;
		double robotAngle = mRobotAngle.getAngleDegrees();

		// Don't need to check Y button, which is 0 degrees, since newAngle set to 0 by default

		if (mController.getBButton()) {
			newAngle = 90;
		}
		else if (mController.getAButton()) {
			newAngle = 180;
		}
		else if (mController.getXButton()) {
			newAngle = 270;
		}

		if (mIsFieldRelative) {
			newAngle -= robotAngle;
		}

		return Vector.createPolar(newAngle, DriveConstants.SwerveSpeeds.NUDGE_MOVE_SPEED);
	}

	/**
	 * quadratic control of right trigger axis
	 * 
	 * @return trigger axis squared
	 */
	private double getStickLinearVel() {
		double speed = mController.getTriggerAxis(Hand.kRight);
		SmartDashboard.putNumber("Right Trigger Axis", speed);
		speed = Math.pow(speed, 2) * DriveConstants.SwerveSpeeds.SPEED_MULT;
		// quadratic control, finer control of lower speeds
		return speed;
	}

	/**
	 * Angular velocity calculated with the right joystick
	 * 
	 * @return angular velocity for swerve drive
	 */
	private double getStickAngularVel() {
		double joystickValue = mController.getX(Hand.kRight);
		SmartDashboard.putNumber("Right Joystick X", joystickValue);

		if (Math.abs(joystickValue) < DriveConstants.MIN_DIRECTION_MAG) {
			return 0;
		}
		double angularVel = joystickValue * Math.abs(joystickValue);
		angularVel *= DriveConstants.SwerveSpeeds.ANGULAR_SPEED_MULT;
		SmartDashboard.putNumber("Angular Velocity", angularVel);
		return angularVel; // quadratic control for finer movements
	}

	/**
	 * Disables and resets drift compensation
	 */
	private void resetDriftCompensation() {
		mDriftCompensationPID.reset();
	}

	/**
	 * Gets linear velocity state
	 * 
	 * @return bumper --> nudge; no move --> angle only; else --> normal
	 */
	private LinearVelocity getLinearVelocityState() {
		LinearVelocity linVel = LinearVelocity.NONE;
		if (getLetterPressed()) {
			linVel = LinearVelocity.NUDGE;
		} 
		else if (getStickLinearVel() < DriveConstants.MIN_LINEAR_VELOCITY
				&& getStickMag(Hand.kLeft) > DriveConstants.MIN_DIRECTION_MAG) {
			linVel = LinearVelocity.ANGLE_ONLY;
		} 
		else if (getStickLinearVel() > DriveConstants.MIN_LINEAR_VELOCITY) {
			linVel = LinearVelocity.NORMAL;
		}

		if (linVel != mLinearVel) {
			mPrevLinearVel = mLinearVel;
		}
		
		return linVel;
	}

	/**
	 * Gets rotational velocity state
	 * 
	 * @return bumper --> nudge; no move --> none; else --> normal
	 */
	private RotationalVelocity getRotationalVelocityState() {
		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_RIGHT)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_FRONT) 
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_LEFT)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_BOTTOM)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_SIDE)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_TOP)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LOADING_STATION)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_BOTTOM)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_SIDE)
		 || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_TOP)){
			return RotationalVelocity.SECONDARY;
		 }
		else if (mController.getPOV() >= 0) {
			return RotationalVelocity.POV;
		} 
		else if (mController.getBumper(Hand.kRight) || mController.getBumper(Hand.kLeft)) {
			return RotationalVelocity.NUDGE;
		} 
		else if (getStickAngularVel() != 0) {
			return RotationalVelocity.NORMAL;
		}
		return RotationalVelocity.NONE;
	}

	private boolean getLetterPressed() {
		return (mController.getAButton() || mController.getBButton() || mController.getXButton()
				|| mController.getYButton());
	}

	// private boolean getGyroGoalAngle() {
	// 	if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_BOTTOM)){
	// 		secondaryJoystickAngle = 331.3;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_SIDE) || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_RIGHT)){
	// 		secondaryJoystickAngle = 270;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEFT_ROCKET_TOP)){
	// 		secondaryJoystickAngle = 208.7;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_BOTTOM)){
	// 		secondaryJoystickAngle = 28.7;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_SIDE) || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_LEFT)){
	// 		secondaryJoystickAngle = 90;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.RIGHT_ROCKET_TOP)){
	// 		secondaryJoystickAngle = 151.3;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LOADING_STATION)){
	// 		secondaryJoystickAngle = 180;
	// 	}
	// 	else if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CARGO_SHIP_FRONT)){
	// 		secondaryJoystickAngle = 0;
	// 	}
	// }

	/**
	 * Calculate angular velocity to turn to a certain angle
	 * 
	 * @param setpointAngle
	 *            angle to turn to
	 * @return angular velocity required to turn to the angle
	 */
	public double getAngularPIDVel(double setpointAngle) {
		mGyroPID.setSetpoint(setpointAngle);

		mGyroPID.enable();

		double vel = mGyroOutput.getVal();

		SmartDashboard.putNumber("Gyro PID Setpoint:", mGyroPID.getSetpoint());
		SmartDashboard.putNumber("Gyro PID Output:", vel);

		return vel;
	}

	public Vector getDesiredRobotVel() {
		return mDesiredRobotVel;
	}

	public double getDesiredAngularVel() {
		return mDesiredAngularVel;
	}

	private void pidInit() {
		mGyroOutput = new GenericPIDOutput();
		mGyroPID = new LocalPIDController(DriveConstants.ActualRobot.GYRO_P, DriveConstants.ActualRobot.GYRO_I,
		DriveConstants.ActualRobot.GYRO_D, mRobotAngle, mGyroOutput);
		//mGyroPID = new PIDController(DriveConstants.ActualRobot.GYRO_P, DriveConstants.ActualRobot.GYRO_I,
			//	DriveConstants.ActualRobot.GYRO_D, mRobotAngle, mGyroOutput);
		mGyroPID.setInputRange(0, 360.0);
		mGyroPID.setOutputRange(-DriveConstants.ActualRobot.GYRO_MAX_SPEED, DriveConstants.ActualRobot.GYRO_MAX_SPEED);
		mGyroPID.setAbsoluteTolerance(DriveConstants.ActualRobot.GYRO_TOLERANCE);
		mGyroPID.setContinuous(true);
		mGyroPID.disable();

		//TODO add izone, deadband

		mDriftCompensationOutput = new GenericPIDOutput();
		mDriftCompensationPID = new LocalPIDController(DriveConstants.ActualRobot.DRIFT_COMP_P,
				DriveConstants.ActualRobot.DRIFT_COMP_I, DriveConstants.ActualRobot.DRIFT_COMP_D, mRobotAngle,
				mDriftCompensationOutput);
		mDriftCompensationPID.setOutputRange(-DriveConstants.ActualRobot.DRIFT_COMP_MAX, DriveConstants.ActualRobot.DRIFT_COMP_MAX);
		mDriftCompensationPID.setInputRange(0, 360);
		mDriftCompensationPID.setContinuous(true);
		mDriftCompensationPID.setSetpoint(0);
		mDriftCompensationPID.disable();
	}

	public boolean allWheelsInRange(double pAngle) {
		for (int i = 0; i < 4; i++) {
			if (!mWheels[i].isInRange(pAngle)) {
				return false;
			}
		}
		return true;
	}

	public boolean gyroInRange() {
		return mGyroPID.onTarget();
	}

	public double getRobotAngle() {
		return mRobotAngle.getAngleDegrees();
	}
	
	public Wheel[] returnWheels() {
		return mWheels;
	}
	
}