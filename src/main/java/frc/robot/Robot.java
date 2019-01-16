package frc.robot;

import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import autonomous.AutonomousRoutineType;
import autonomous.commands.AutonomousCommand;
import autonomous.routines.DefaultRoutine;
import autonomous.routines.DoNothingRoutine;
import constants.DriveConstants;
import constants.LeadscrewConstants;
import constants.Ports;
import constants.RunConstants;
import constants.RobotState;
import constants.CameraConstants;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.driving.*;
import robotcode.pneumatics.*;
import robotcode.camera.*;
import robotcode.systems.HatchIntake;
import robotcode.systems.Leadscrew;
import sensors.LeadscrewEncoder;
import sensors.RobotAngle;
import sensors.TalonAbsoluteEncoder;

@SuppressWarnings("deprecation")
public class Robot extends SampleRobot {

	// **********//
	// VARIABLES //
	// **********//

	// controllers
	private XboxController mController;
	private Joystick mJoystick;

	// drive train
	private DriveTrain mDriveTrain;
	private Wheel[] mWheel = new Wheel[4];
	private WPI_TalonSRX[] mTurn = new WPI_TalonSRX[4];
	private WPI_TalonSRX[] mDrive = new WPI_TalonSRX[4];
	private TalonAbsoluteEncoder[] mEncoder = new TalonAbsoluteEncoder[4];

	// gyro
	private AHRS mNavX;
	private RobotAngle mRobotAngle;

	// hatch intake
	private HatchIntake mHatchIntake;
	private DoubleSolenoidReal mHatchRotaryPiston;
	private DoubleSolenoidReal mHatchLinearPiston;

	// leadscrew
	private WPI_TalonSRX mLeadscrewTalon;
	private LeadscrewEncoder mLeadscrewEncoder;
	private Leadscrew mLeadscrew;

	// limelight
	private Limelight mHatchCamera;

	// PDP and compressor
	private PowerDistributionPanel mPDP;
	private Compressor mCompressor;

	// autonomous setup
	private AutonomousRoutineType mAutonomousRoutine = AutonomousRoutineType.DEFAULT;

	// game setup
	private boolean mInGame = false;
	private long mGameStartMillis;
	RobotState mCurrentState = RobotState.DEFAULT;




	
	// *************//
	// GENERAL CODE //
	// *************//
	public Robot() {
	}

	public void test() {
	}

	public void endGame() {
	}

	public void startGame() {
		if (!mInGame) {
			mGameStartMillis = System.currentTimeMillis();

			if (RunConstants.RUNNING_PNEUMATICS) {
				mCompressor.start();
			} else {
				mCompressor.stop();
			}

			mInGame = true;
		}
	}

	public void robotInit() {

		mController = new XboxController(Ports.XBOX);
		mNavX = new AHRS(Ports.NAVX);
		mPDP = new PowerDistributionPanel();

		if (RunConstants.RUNNING_DRIVE) {
			driveInit();
		}

		if (RunConstants.SECONDARY_JOYSTICK) {
			mJoystick = new Joystick(Ports.JOYSTICK);
		}

		if (RunConstants.RUNNING_HATCH) {
			hatchIntakeInit();
		}

		if (RunConstants.RUNNING_LEADSCREW) {
			leadscrewInit();
		}

		if (RunConstants.RUNNING_CAMERA) {
			UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
			camera.setResolution(240, 180);
			camera.setFPS(30);
		}

		mCompressor = new Compressor(Ports.COMPRESSOR);
	}

	public void autonomous() {
		// select auto commands
		ArrayList<AutonomousCommand> autonomousCommands;

		if (mAutonomousRoutine == AutonomousRoutineType.DEFAULT) {
			autonomousCommands = (new DefaultRoutine(this)).getAutonomousCommands();
		} else {
			autonomousCommands = (new DoNothingRoutine()).getAutonomousCommands();
		}

		// start game
		startGame();

		// initialize step variables
		int currentStep = 0;
		int previousStep = -1;

		while (isAutonomous() && isEnabled()) {
			SmartDashboard.putNumber("Autonomous step", currentStep);

			if (currentStep < autonomousCommands.size()) {
				AutonomousCommand command = autonomousCommands.get(currentStep);

				if (currentStep != previousStep) {
					command.startup();
					previousStep = currentStep;
				}

				boolean moveToNextStep = command.runCommand();
				if (moveToNextStep) {
					currentStep++;
				}
			} // else we're done with auto

			Timer.delay(0.005);
		}
	}

	public void operatorControl() {
		// start game, again
		startGame();

		while (isOperatorControl() && isEnabled()) {
			if (RunConstants.RUNNING_DRIVE) {
				swerveDrive();
			}

			if (RunConstants.RUNNING_HATCH) {
				mHatchIntake.enactMovement();
			}

			if (RunConstants.RUNNING_LEADSCREW) {
				mLeadscrew.enactMovement();
				SmartDashboard.putBoolean("Forward Limit Switch Closed", mLeadscrewTalon.getSensorCollection().isFwdLimitSwitchClosed());
				SmartDashboard.putBoolean("Reverse Limit Switch Closed", mLeadscrewTalon.getSensorCollection().isRevLimitSwitchClosed());
				SmartDashboard.putNumber("Leadscrew raw ticks", mLeadscrewEncoder.getRawTicks());
				SmartDashboard.putNumber("leadscrew cooked ticks", mLeadscrewEncoder.getTicksFromEnd());
				SmartDashboard.putNumber("Leadscrew inches", mLeadscrewEncoder.getDistanceInInchesFromEnd());
				SmartDashboard.putNumber("Limelight angle",	NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
				SmartDashboard.putNumber("Limelight error", CameraConstants.LimelightConstants.HEIGHT * Math.tan(Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0))));
				SmartDashboard.putNumber("Leadscrew motor goal ticks", mLeadscrewTalon.getClosedLoopTarget());
				SmartDashboard.putNumber("Leadscrew motor goal inches", LeadscrewEncoder.leadscrewTickToInch(mLeadscrewTalon.getClosedLoopTarget()));
				SmartDashboard.putNumber("Leadscrew motor output", mLeadscrewTalon.getMotorOutputPercent());
				SmartDashboard.putNumber("Leadscrew error", mLeadscrewTalon.getClosedLoopError());
			}

			if (RunConstants.RUNNING_EVERYTHING) {
				doWork();
			}

			// put info on SmartDashboard
			SmartDashboard.putString("Current State", mCurrentState.toString());
			if (RunConstants.RUNNING_DRIVE) {
				for (int i = 0; i < 4; i++) {
					SmartDashboard.putNumber("Motor Output Percent " + i, mDrive[i].getMotorOutputPercent());
				}
			}

			Timer.delay(0.005); // wait for a motor update time
		}
	}

	private void doWork() {
		switch (mCurrentState) {
		case DEFAULT:
			doSomeAction();
			break;
		default:
			throw new RuntimeException("Unknown state");
		}

		SmartDashboard.putString("Current State", mCurrentState.name());
	}

	private void doSomeAction() {
		// Do some action... move to a different state?
	}

	public void disabled() {
		endGame();

		while (this.isDisabled()) {
			if (RunConstants.SECONDARY_JOYSTICK) {

				if (mJoystick.getTriggerPressed()) {
					// rotate autonomous routines to select which one to start with:
					if (mAutonomousRoutine == AutonomousRoutineType.DEFAULT) {
						mAutonomousRoutine = AutonomousRoutineType.DO_NOTHING;
					} else if (mAutonomousRoutine == AutonomousRoutineType.DO_NOTHING) {
						mAutonomousRoutine = AutonomousRoutineType.DEFAULT;
					}
				}
			}
		}

		// SmartDashboard.putString("AUTO ROUTINE:", mAutonomousRoutine.toString());
		Timer.delay(0.005); // wait for a motor update time
	}





	// ***************//
	// INITIALIZATION //
	// ***************//
	public void driveInit() {
		int turnPort, turnOffset, drivePort, iZone, rotTol;
		double P_PID, I_PID, D_PID;
		boolean turnEncoderReversed, turnReversed, driveReversed;

		for (int i = 0; i < 4; i++) {
			if (RunConstants.IS_PROTOTYPE) { // determine values based on if prototype or real robot being used
				turnPort = Ports.PrototypeRobot.TURN[i];
				turnEncoderReversed = DriveConstants.PrototypeRobot.ENCODER_REVERSED[i];
				turnReversed = DriveConstants.PrototypeRobot.TURN_INVERTED[i];
				turnOffset = DriveConstants.PrototypeRobot.OFFSETS[i];
				driveReversed = DriveConstants.PrototypeRobot.DRIVE_INVERTED[i];
				drivePort = Ports.PrototypeRobot.DRIVE[i];
				P_PID = DriveConstants.PrototypeRobot.ROTATION_P[i];
				I_PID = DriveConstants.PrototypeRobot.ROTATION_I[i];
				D_PID = DriveConstants.PrototypeRobot.ROTATION_D[i];
				iZone = DriveConstants.PrototypeRobot.ROTATION_IZONE[i];
				rotTol = DriveConstants.PrototypeRobot.ROTATION_TOLERANCE[i];
			} else {
				turnPort = Ports.ActualRobot.TURN[i];
				turnEncoderReversed = DriveConstants.ActualRobot.ENCODER_REVERSED[i];
				turnReversed = DriveConstants.ActualRobot.TURN_INVERTED[i];
				turnOffset = DriveConstants.ActualRobot.OFFSETS[i];
				driveReversed = DriveConstants.ActualRobot.DRIVE_INVERTED[i];
				drivePort = Ports.ActualRobot.DRIVE[i];
				P_PID = DriveConstants.ActualRobot.ROTATION_P[i];
				I_PID = DriveConstants.ActualRobot.ROTATION_I[i];
				D_PID = DriveConstants.ActualRobot.ROTATION_D[i];
				iZone = DriveConstants.ActualRobot.ROTATION_IZONE[i];
				rotTol = DriveConstants.ActualRobot.ROTATION_TOLERANCE[i];
			}

			// initialize turn motors and set values:
			mTurn[i] = new WPI_TalonSRX(turnPort);
			mTurn[i].configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
			mTurn[i].setNeutralMode(NeutralMode.Brake);
			mTurn[i].setSensorPhase(turnEncoderReversed);
			mTurn[i].setInverted(turnReversed);
			mTurn[i].config_kP(0, P_PID, 10);
			mTurn[i].config_kI(0, I_PID, 10);
			mTurn[i].config_kD(0, D_PID, 10);
			mTurn[i].config_IntegralZone(0, iZone, 10);
			mTurn[i].configAllowableClosedloopError(0, rotTol, 10);
			mTurn[i].configPeakOutputForward(1, 10);
			mTurn[i].configPeakOutputReverse(-1, 10);

			// initialize drive motors and set values:
			mDrive[i] = new WPI_TalonSRX(drivePort);
			mDrive[i].setInverted(driveReversed);
			mDrive[i].setNeutralMode(NeutralMode.Brake);
			mDrive[i].configPeakOutputForward(1, 10);
			mDrive[i].configPeakOutputReverse(-1, 10);
			mDrive[i].configPeakCurrentDuration(1000, 10);
			mDrive[i].configPeakCurrentLimit(150, 10);
			mDrive[i].configContinuousCurrentLimit(80, 10);
			mDrive[i].enableCurrentLimit(true);

			// initialize turn motors' encoders, as well as wheels:
			mEncoder[i] = new TalonAbsoluteEncoder(mTurn[i], ResourceFunctions.tickToAngle(turnOffset));
			mWheel[i] = new Wheel(mTurn[i], mDrive[i], mEncoder[i]);
		}

		mRobotAngle = new RobotAngle(mNavX, false, 0);
		mDriveTrain = new DriveTrain(mWheel, mController, mRobotAngle);
	}

	public void hatchIntakeInit() {
		mHatchRotaryPiston = new DoubleSolenoidReal(Ports.ActualRobot.HATCH_ROTARY_SOLENOID_IN,
				Ports.ActualRobot.HATCH_ROTARY_SOLENOID_OUT);
		mHatchLinearPiston = new DoubleSolenoidReal(Ports.ActualRobot.HATCH_LINEAR_SOLENOID_IN,
				Ports.ActualRobot.HATCH_LINEAR_SOLENOID_OUT);

		mHatchCamera = new Limelight();
		mHatchCamera.setPipeline(0);

		mHatchIntake = new HatchIntake(mHatchRotaryPiston, mHatchLinearPiston, mJoystick);
	}

	public void leadscrewInit() {
		mLeadscrewTalon = new WPI_TalonSRX(Ports.ActualRobot.LEADSCREW);

		mLeadscrewTalon.setInverted(LeadscrewConstants.REVERSED);
		mLeadscrewTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, 0, 10);
		mLeadscrewTalon.setSensorPhase(LeadscrewConstants.ENCODER_REVERSED);

		mLeadscrewTalon.setNeutralMode(NeutralMode.Brake);
		mLeadscrewTalon.config_kP(0, LeadscrewConstants.PID.LEADSCREW_P, 10);
		mLeadscrewTalon.config_kI(0, LeadscrewConstants.PID.LEADSCREW_I, 10);
		mLeadscrewTalon.config_kD(0, LeadscrewConstants.PID.LEADSCREW_D, 10);
		mLeadscrewTalon.config_IntegralZone(0, LeadscrewConstants.PID.LEADSCREW_IZONE, 10);
		mLeadscrewTalon.configAllowableClosedloopError(0, LeadscrewConstants.PID.LEADSCREW_TOLERANCE, 10);

		mLeadscrewEncoder = new LeadscrewEncoder(mLeadscrewTalon, LeadscrewConstants.OFFSET);

		mLeadscrew = new Leadscrew(mLeadscrewTalon, mLeadscrewEncoder, mHatchCamera, mJoystick);
	}

<<<<<<< HEAD
		mHatchCamera = new Limelight();
		mHatchCamera.setPipeline(CameraConstants.LimelightConstants.PIPELINE);
=======
>>>>>>> 3a475c6c639caafe6353af3f0ef8b6491c96c039




	// ******//
	// DRIVE //
	// ******//
	private void tankDrive() {
		if (RunConstants.RUNNING_DRIVE) {
			mDriveTrain.driveTank();
		}
	}

	private void crabDrive() {
		if (RunConstants.RUNNING_DRIVE) {
			mDriveTrain.driveCrab();
		}
	}

	private void swerveDrive() {
		if (RunConstants.RUNNING_DRIVE) {
			mDriveTrain.driveSwerve();
		}
	}

	public DriveTrain getDriveTrain() {
		return mDriveTrain;
	}





	// ********//
	// LOGGING //
	// ********//
	private void addLogValueDouble(StringBuilder pLogString, double pVal) {
		pLogString.append(pVal);
		pLogString.append(",");
	}

	private void addLogValueInt(StringBuilder pLogString, int pVal) {
		pLogString.append(pVal);
		pLogString.append(",");
	}

	private void addLogValueLong(StringBuilder pLogString, long pVal) {
		pLogString.append(pVal);
		pLogString.append(",");
	}

	private void addLogValueBoolean(StringBuilder pLogString, boolean pVal) {
		pLogString.append(pVal ? "1" : "0");
		pLogString.append(",");
	}

	private void addLogValueString(StringBuilder pLogString, String pVal) {
		pLogString.append(pVal);
		pLogString.append(",");
	}

	private void addLogValueEndDouble(StringBuilder pLogString, double pVal) {
		pLogString.append(pVal);
		pLogString.append("\n");
	}

	private void addLogValueEndInt(StringBuilder pLogString, int pVal) {
		pLogString.append(pVal);
		pLogString.append("\n");
	}

	private void addLogValueEndLong(StringBuilder pLogString, long pVal) {
		pLogString.append(pVal);
		pLogString.append("\n");
	}

	private void addLogValueEndBoolean(StringBuilder pLogString, boolean pVal) {
		pLogString.append(pVal ? "1" : "0");
		pLogString.append("\n");
	}

	private void addLogValueEndString(StringBuilder pLogString, String pVal) {
		pLogString.append(pVal);
		pLogString.append("\n");
	}

	public void log() {
		long time = System.currentTimeMillis();
		long timeElapsed = time - mGameStartMillis;

		SmartDashboard.putBoolean("Game Has Started:", mInGame);
		SmartDashboard.putNumber("Time Game Started:", mGameStartMillis);
		SmartDashboard.putNumber("Time Elapsed:", timeElapsed);

		StringBuilder logString = new StringBuilder();

		// for now it is one frame per line
		addLogValueInt(logString, (int) timeElapsed);

		addLogValueBoolean(logString, mController.getYButton());
		addLogValueBoolean(logString, mController.getBButton());
		addLogValueBoolean(logString, mController.getAButton());
		addLogValueBoolean(logString, mController.getXButton());
		addLogValueBoolean(logString, mController.getBumper(Hand.kLeft));
		addLogValueBoolean(logString, mController.getBumper(Hand.kRight));
		addLogValueDouble(logString, mController.getTriggerAxis(Hand.kLeft));
		addLogValueDouble(logString, mController.getTriggerAxis(Hand.kRight));
		addLogValueInt(logString, mController.getPOV());
		addLogValueBoolean(logString, mController.getStartButton());
		addLogValueBoolean(logString, mController.getBackButton());
		addLogValueDouble(logString, mController.getX(Hand.kLeft));
		addLogValueDouble(logString, mController.getY(Hand.kLeft));
		addLogValueDouble(logString, mController.getX(Hand.kRight));
		addLogValueDouble(logString, mController.getY(Hand.kRight));

		if (RunConstants.SECONDARY_JOYSTICK) {
			for (int i = 1; i < 12; i++) {
				addLogValueBoolean(logString, mJoystick.getRawButton(i));
			}
		}

		if (RunConstants.RUNNING_DRIVE) {
			for (int i = 0; i < 4; i++) {
				addLogValueDouble(logString, mTurn[i].getOutputCurrent());
				addLogValueDouble(logString, mDrive[i].getOutputCurrent());

				addLogValueDouble(logString, mTurn[i].getMotorOutputVoltage());
				addLogValueDouble(logString, mDrive[i].getMotorOutputVoltage());

				addLogValueDouble(logString, mEncoder[i].getAngleDegrees());
			}

			addLogValueDouble(logString, mDriveTrain.getDesiredRobotVel().getMagnitude());
			addLogValueDouble(logString, mDriveTrain.getDesiredRobotVel().getAngle());
			addLogValueDouble(logString, mDriveTrain.getDesiredAngularVel());
		}

		if (RunConstants.RUNNING_PNEUMATICS) {
			addLogValueDouble(logString, mCompressor.getCompressorCurrent());
		}
		addLogValueDouble(logString, mPDP.getTotalCurrent());
		addLogValueDouble(logString, mPDP.getVoltage());

		addLogValueString(logString, mCurrentState.toString());

		addLogValueEndDouble(logString, mRobotAngle.getAngleDegrees());

		SmartDashboard.putString("LogString", logString.toString());
	}
}
