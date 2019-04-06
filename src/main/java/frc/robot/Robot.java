package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import autonomous.AutonomousRoutineType;
import constants.CameraConstants;
import constants.ClimberConstants;
import constants.DriveConstants;
import constants.JoystickConstants;
import constants.LeadscrewConstants;
import constants.Ports;
import constants.RobotState;
import constants.RunConstants;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import resource.ResourceFunctions;
import robotcode.LocalJoystick;
import robotcode.camera.Limelight;
import robotcode.driving.DriveTrain;
import robotcode.driving.Wheel;
import robotcode.pneumatics.DoubleSolenoidReal;
import robotcode.pneumatics.SingleSolenoidReal;
import robotcode.pneumatics.SolenoidInterface;
import robotcode.systems.BallIntake;
import robotcode.systems.ClimberPiston;
import robotcode.systems.ClimberSpark;
import robotcode.systems.HatchIntake;
import robotcode.systems.Intake;
import robotcode.systems.Leadscrew;
import sensors.LeadscrewEncoder;
import sensors.RobotAngle;
import sensors.TalonAbsoluteEncoder;
import sensors.UltrasonicSensor;

@SuppressWarnings("deprecation")
public class Robot extends SampleRobot {

	// **********//
	// VARIABLES //
	// **********//

	// controllers
	private XboxController mController;
	private LocalJoystick mJoystick;
	private Joystick mClimbJoystick;

	// drive train
	private DriveTrain mDriveTrain;
	private Wheel[] mWheel = new Wheel[4];
	private CANSparkMax[] mDrive = new CANSparkMax[4];
	private CANEncoder[] mDriveEncoders = new CANEncoder[4]; 
	private WPI_TalonSRX[] mTurn = new WPI_TalonSRX[4];
	private TalonAbsoluteEncoder[] mEncoder = new TalonAbsoluteEncoder[4];

	// gyro
	private AHRS mNavX;
	private RobotAngle mRobotAngle;

	// hatch intake
	private HatchIntake mHatchIntake;
	private SolenoidInterface mHatchRotaryPiston, mHatchLinearPiston;

	// ball intake
	private BallIntake mBallIntake;
	private SolenoidInterface mBallRotary, mBallLock, mBallRetain;
	private UltrasonicSensor mUltraRight, mUltraLeft;

	// leadscrew
	private WPI_TalonSRX mLeadscrewTalon;
	private LeadscrewEncoder mLeadscrewEncoder;
	private Leadscrew mLeadscrew;

	// cameras
	private Limelight mHatchCamera;

	// intake
	private Intake mIntake;

	// climber
	private WPI_TalonSRX mDriveClimbTalon0, mDriveClimbTalon1;
	private DoubleSolenoidReal mFrontClimbPiston, mBackClimbPiston, mFrontBreak, mBackBreak;
	private ClimberPiston mClimber;

	// LEDs
	private Spark mBlinkin;

	// PDP and compressor
	private PowerDistributionPanel mPDP;
	private Compressor mCompressor;

	// autonomous setup
	private AutonomousRoutineType mAutonomousRoutine = AutonomousRoutineType.DEFAULT;

	// game setup
	private boolean mInGame = false;
	private long mGameStartMillis, mTimeLastCycleStarted;

	RobotState mCurrentState = RobotState.INITIAL_HOLDING_HATCH;

	// *************//
	// GENERAL CODE //
	// *************//
	public Robot() {

	}

	public void test() {

	}

	public void endGame() {
		if(RunConstants.LOGGING){
			SmartDashboard.putString("DashboardCommand", "EndRecording");
		}
	}

	public void startGame() {
		if (!mInGame) {
			mGameStartMillis = System.currentTimeMillis();

			if(RunConstants.LOGGING){
				createHeaderString();
				SmartDashboard.putString("DashboardCommand", "StartRecording");
			}

			if(RunConstants.SECONDARY_JOYSTICK){
				SmartDashboard.putString("JoystickCommand", "StartUpdating");
				SmartDashboard.putString("LEDCommand", "ENABLED");
			}

			if (RunConstants.RUNNING_PNEUMATICS) {
				mCompressor.start();
			}

			if (RunConstants.RUNNING_HATCH) {
				mHatchIntake.expand();
				mHatchIntake.in();
			}

			if (RunConstants.RUNNING_BALL) {
				mBallIntake.lock();
				mBallIntake.retain();
				mBallIntake.backward();
			}

			if (RunConstants.RUNNING_LEADSCREW) {
				mLeadscrew.leadscrewInitialZero();
				mLeadscrew.setPosition(LeadscrewConstants.MIDDLE);
				while (dummy()) {
				}
			}

			if (RunConstants.RUNNING_CLIMBER) {
				mFrontClimbPiston.set(ClimberConstants.FRONT_LEGS_UP);
				mBackClimbPiston.set(ClimberConstants.BACK_LEGS_UP);
			}

			mInGame = true;
		}
	}

	public boolean dummy() {
		int error = mLeadscrewEncoder.getError((int) mLeadscrewTalon.getClosedLoopTarget());
		return error > LeadscrewConstants.PID.LEADSCREW_TOLERANCE;
	}

	public void robotInit() {

		mController = new XboxController(Ports.XBOX);
		mJoystick = new LocalJoystick(Ports.JOYSTICK);
		mClimbJoystick = new Joystick(Ports.CLIMB_JOYSTICK);

		mNavX = new AHRS(Ports.NAVX);

		mPDP = new PowerDistributionPanel();
		mCompressor = new Compressor(Ports.COMPRESSOR);

		mBlinkin = new Spark(Ports.ActualRobot.BLINKIN_CHANNEL);

		if (RunConstants.RUNNING_DRIVE) {
			driveInit();
		}

		if (RunConstants.RUNNING_HATCH) {
			hatchIntakeInit();
		}

		if (RunConstants.RUNNING_BALL) {
			ballInit();
		}

		if (RunConstants.RUNNING_CAMERA) {
			cameraInit();
		}

		if (RunConstants.RUNNING_LEADSCREW) {
			leadscrewInit();
		}

		if (RunConstants.RUNNING_LEADSCREW && RunConstants.RUNNING_HATCH && RunConstants.RUNNING_BALL) {
			intakeInit();
		}

		if (RunConstants.RUNNING_CLIMBER) {
			climberInit();
		}

	}

	public void autonomous() {
		// select auto commands
		// ArrayList<AutonomousCommand> autonomousCommands;

		// if (mAutonomousRoutine == AutonomousRoutineType.DEFAULT) {
		// 	autonomousCommands = (new DefaultRoutine(this)).getAutonomousCommands();
		// } else {
		// 	autonomousCommands = (new DoNothingRoutine()).getAutonomousCommands();
		// }

		// start game
		startGame();
		NetworkTableInstance.getDefault().setUpdateRate(0.015);

		mTimeLastCycleStarted = System.currentTimeMillis();
		long timeCycleStart;
		long lastCycleTime;

		// initialize step variables
		int currentStep = 0;
		int previousStep = -1;

		while (isAutonomous() && isEnabled()) {

			timeCycleStart = System.currentTimeMillis();
			lastCycleTime = timeCycleStart - mTimeLastCycleStarted;
			mTimeLastCycleStarted = timeCycleStart;
			SmartDashboard.putNumber("Cycle Time", lastCycleTime);
			SmartDashboard.putNumber("CycleStart", timeCycleStart);

			if(RunConstants.LOGGING){
				log();
			}

			// if (RunConstants.RUNNING_DRIVE) {
			// 	//swerveDrive();
			// 	for (int i = 0; i < 4; i++) {
			// 		SmartDashboard.putNumber("Motor Output Percent " + i, mDrive[i].get());
			// 		SmartDashboard.putNumber("Motor current " + i, mTurn[i].getOutputCurrent());
			// 		SmartDashboard.putNumber("Wheel offset " + i, mTurn[i].getSelectedSensorPosition() % 4096);
			// 		SmartDashboard.putNumber("Wheel position " + i, mTurn[i].getSelectedSensorPosition());
			// 	}
			// }
			if(RunConstants.RUNNING_BALL){
				// SmartDashboard.putBoolean("Ultrasonic value", mBallIntake.isHoldingBall());
				// SmartDashboard.putNumber("Ultrasonic voltage", mBallIntake.getUltrasonicRight().getValue());
			}
		
			// put info on SmartDashboard

			if (!RunConstants.SECONDARY_JOYSTICK) { // only do this if we're using the logitech attack 3
				mJoystick.updateProfile();
				SmartDashboard.putNumber("JOYSTICK PROFILE NUMBER", mJoystick.getProfile());
			} 

			if (RunConstants.RUNNING_EVERYTHING){
				doWork();
				SmartDashboard.putBoolean("BallSensed", mBallIntake.isHoldingBall());
				SmartDashboard.putBoolean("LeadscrewAligned", mLeadscrew.isInRange());
				SmartDashboard.putNumber("CLIMB front(?) current draw", mPDP.getCurrent(2));
				SmartDashboard.putNumber("CLIMB back(?) current draw", mPDP.getCurrent(3));
				SmartDashboard.putNumber("CLIMB other back current draw", mPDP.getCurrent(12));
			}

			// SmartDashboard.putNumber("Autonomous step", currentStep);

			// if (currentStep < autonomousCommands.size()) {
			// 	AutonomousCommand command = autonomousCommands.get(currentStep);

			// 	if (currentStep != previousStep) {
			// 		command.startup();
			// 		previousStep = currentStep;
			// 	}

			// 	boolean moveToNextStep = command.runCommand();
			// 	if (moveToNextStep) {
			// 		currentStep++;
			// 	}
			// } // else we're done with auto

			Timer.delay(0.005);
		}
	}

	public void operatorControl() {
		// start game, again
		startGame();
		NetworkTableInstance.getDefault().setUpdateRate(0.015);

		mTimeLastCycleStarted = System.currentTimeMillis();
		long timeCycleStart;
		long lastCycleTime;
		SmartDashboard.putString("JoystickCommand", "StartUpdating");

		long lastUpdate = 0;
		long thisUpdate = NetworkTableInstance.getDefault().getConnections()[0].last_update;

		while (isOperatorControl() && isEnabled()) {
			
			timeCycleStart = System.currentTimeMillis();
			lastCycleTime = timeCycleStart - mTimeLastCycleStarted;
			mTimeLastCycleStarted = timeCycleStart;
			SmartDashboard.putNumber("Cycle Time", lastCycleTime);
			SmartDashboard.putNumber("CycleStart", timeCycleStart);


			thisUpdate = NetworkTableInstance.getDefault().getConnections()[0].last_update;
			SmartDashboard.putNumber("NT update diff", thisUpdate - lastUpdate);
			lastUpdate = thisUpdate;


			if(RunConstants.LOGGING){
				log();
			}

			if(RunConstants.RUNNING_CAMERA){
				mHatchCamera.setStreamSecondary();
			}

			// if (RunConstants.RUNNING_DRIVE) {
			// 	//swerveDrive();
			// 	for (int i = 0; i < 4; i++) {
			// 		SmartDashboard.putNumber("Motor Output Percent " + i, mDrive[i].get());
			// 		SmartDashboard.putNumber("Motor current " + i, mTurn[i].getOutputCurrent());
			// 		SmartDashboard.putNumber("Wheel offset " + i, mTurn[i].getSelectedSensorPosition() % 4096);
			// 		SmartDashboard.putNumber("Wheel position " + i, mTurn[i].getSelectedSensorPosition());
			// 	}
				
			// }

			// hatch intake without leadscrew
			if (RunConstants.RUNNING_HATCH && !RunConstants.RUNNING_LEADSCREW && !RunConstants.RUNNING_EVERYTHING && !RunConstants.RUNNING_BALL) {
				mHatchIntake.enactMovement();
			}

			// ball
			else if (RunConstants.RUNNING_BALL && !RunConstants.RUNNING_EVERYTHING && !RunConstants.RUNNING_HATCH && !RunConstants.RUNNING_LEADSCREW) {
				
				mIntake.intakeBall();
				mIntake.scoreBallHigh();
				mIntake.scoreBallLow();
				mIntake.holdingBall();
				mIntake.idle();

			}

			// leadscrew without hatch intake or ball
			else if (RunConstants.RUNNING_LEADSCREW && !RunConstants.RUNNING_HATCH && !RunConstants.RUNNING_BALL
					&& !RunConstants.RUNNING_EVERYTHING) {
				mLeadscrew.enactMovement();
				
				// SmartDashboard.putNumber("Limelight angle",
				// NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0));
				// SmartDashboard.putNumber("Limelight error",
				// CameraConstants.LimelightConstants.HEIGHT *
				// Math.tan(Math.toRadians(NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0))));
			}

			// all intake things but not states -- for testing
			else if (RunConstants.RUNNING_HATCH && RunConstants.RUNNING_LEADSCREW && RunConstants.RUNNING_BALL && !RunConstants.RUNNING_EVERYTHING) {
				mIntake.enactMovement();
			}

			else if (RunConstants.RUNNING_CLIMBER && !RunConstants.RUNNING_EVERYTHING) {
				mClimber.manualClimb();
				// SmartDashboard.putString("CLIMB Shifter", mClimbShifter.get().toString());
				SmartDashboard.putString("Front CLIMB", mFrontClimbPiston.get().toString());
				SmartDashboard.putString("Back CLIMB", mBackClimbPiston.get().toString());
				SmartDashboard.putNumber("robot angle", mNavX.getAngle());
			}

			if (RunConstants.RUNNING_EVERYTHING) {

				//LeadscrewConstants.LEADSCREW_OVERRIDE = mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE);
				doWork();
				SmartDashboard.putBoolean("BallSensed", mBallIntake.isHoldingBall());
				SmartDashboard.putBoolean("LeadscrewAligned", mLeadscrew.isInRange());
			}

			if (!RunConstants.SECONDARY_JOYSTICK) { // only do this if we're using the logitech attack 3
				mJoystick.updateProfile();
				SmartDashboard.putNumber("JOYSTICK PROFILE NUMBER", mJoystick.getProfile());
			}	

			//SmartDashboard.putNumber("BLINKIN value", mBlinkin.get());
			SmartDashboard.putNumber("NAVX pitch", mNavX.getPitch());
			SmartDashboard.putNumber("NAVX roll", mNavX.getRoll());
			SmartDashboard.putNumber("NAVX yaw", mNavX.getYaw());
			SmartDashboard.putString("CLIMBER front break", mFrontBreak.get().toString());
			SmartDashboard.putString("CLIMBER back break", mBackBreak.get().toString());
			Timer.delay(0.005); // wait for a motor update time
		}
	}

	private void doWork() {
		switch (mCurrentState) {
			case INITIAL_HOLDING_HATCH:
				initialHoldingHatch();
				break;
			case INITIAL_HOLDING_BALL:
				initialHoldingBall();
				break;
			case HATCH_SCORE:
				hatchScore();
				break;
			case WAITING_TO_LOAD:
				waitingToLoad();
				break;
			case LOADING_HATCH:
				loadingHatch();
				break;
			case LOADING_BALL:
				loadingBall();
				break;
			case HATCH_PRESCORE:
				hatchPrescore();
				break;
			case BALL_PRESCORE:
				ballPrescore();
				break;
			case BALL_FRONT_SCORE:
				ballFrontScore();
				break;
			case BALL_BACK_SCORE:
				ballBackScore();
				break;
			case DEFENSE:
				defense();
				break;
			case CLIMB:
				climb();
				break;
			case DEFAULT:
				defaultState();
				break;
			default:
				throw new RuntimeException("Unknown state");
		}

		SmartDashboard.putString("Current State", mCurrentState.name());

	}

	/**
	 * initial robot state goes to HATCH_SCORE when we want to score or
	 * WAITING_TO_LOAD if we drop it
	 */
	private void initialHoldingHatch() {
		swerveDrive();

		// when the robot wants to score...
		if (mIntake.holdingHatch() && (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_CARGO)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_CARGO)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_ROCKET)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_ROCKET))) {
			mCurrentState = RobotState.HATCH_SCORE;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
			mCurrentState = RobotState.CLIMB;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}

	}

	private void initialHoldingBall() {

		swerveDrive();
		
		if (mIntake.holdingBall()
				&& (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_CARGO)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_CARGO))) {
			mCurrentState = RobotState.BALL_FRONT_SCORE;
		}

		else if (mIntake.holdingBall()
				&& (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_ROCKET)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_ROCKET))) {
			mCurrentState = RobotState.BALL_BACK_SCORE;
		}

		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)){
			mCurrentState = RobotState.CLIMB;
		}

		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)){
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}

	/**
	 * Score the hatch. when it's done go to WAITING_TO_LOAD state
	 */
	private void hatchScore() {

		swerveDrive();

		// if robot or driver says scoring is done...
		if (mIntake.scorePanel() || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_SCORED_PANEL)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_SCORED_BALL)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
			mCurrentState = RobotState.CLIMB;
		}
		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}

	private long mStartWaitingToLoad = 0;
	private boolean mHasWaitedToLoad = false;

	/**
	 * keep the intake in an idle kinda mode when you get the ball, go to
	 * BALL_PRESCORE when you wanna get the panel, go to LOADING_HATCH
	 */
	private void waitingToLoad() {
		// has_loaded button is pressed and thingy is flipped to ball side
		swerveDrive();

		if (!mHasWaitedToLoad) {
			mStartWaitingToLoad = System.currentTimeMillis();
			mHasWaitedToLoad = true;
		}

		long waitingElapsedMilliseconds = System.currentTimeMillis() - mStartWaitingToLoad;

		if (waitingElapsedMilliseconds > 500) {

			if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.BALL_PANEL_SWITCH)) {
				mBallIntake.lock();
			}
			else {
				mBallIntake.letGo();
			}

			if (mIntake.idle() && mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LOAD_PANEL)) {
				mCurrentState = RobotState.LOADING_HATCH;
				mStartWaitingToLoad = 0;
				mHasWaitedToLoad = false;
			}

			else if (mIntake.idle() && mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LOAD_BALL)) {
				mCurrentState = RobotState.LOADING_BALL;
				mStartWaitingToLoad = 0;
				mHasWaitedToLoad = false;
			}

			if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
				mCurrentState = RobotState.CLIMB;
				mStartWaitingToLoad = 0;
				mHasWaitedToLoad = false;
			}

		}

		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)){
			mCurrentState = RobotState.CLIMB;
		}

	}


	private void loadingBall() {

		swerveDrive();

		if (mIntake.intakeBall() || mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_LOADED_BALL)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_LOADED_PANEL)) {
			mCurrentState = RobotState.BALL_PRESCORE;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
			mCurrentState = RobotState.CLIMB;
		}
		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}


	/**
	 * intake the hatch when either the robot or driver says it's done, go to
	 * HATCH_PRESCORE
	 */
	private void loadingHatch() {

		swerveDrive();

		// either robot or person says the thing has been intaken
		if (mIntake.intakePanel()
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_LOADED_PANEL)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_LOADED_BALL)) {
			mCurrentState = RobotState.HATCH_PRESCORE;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
			mCurrentState = RobotState.CLIMB;
		}
		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}

	private long mTimeStartHatchPrescore = 0;
	private boolean mHasStartedHatchPrescore = false;


	private void hatchPrescore() {

		swerveDrive();

		if (!mHasStartedHatchPrescore) {
			mTimeStartHatchPrescore = System.currentTimeMillis();
			mHasStartedHatchPrescore = true;
		}

		long hatchPrescoreElapsedMilliseconds = System.currentTimeMillis() - mTimeStartHatchPrescore;

		if (hatchPrescoreElapsedMilliseconds > 500) {
			// when the robot wants to score...
			if (mIntake.holdingHatch()
					&& (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_CARGO)
					|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_CARGO)
					|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_ROCKET)
					|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_ROCKET))) {

				mCurrentState = RobotState.HATCH_SCORE;
				mTimeStartHatchPrescore = 0;
				mHasStartedHatchPrescore = false;
			}


			// if we accidentally drop the panel... TODO check transition
		}

		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)){
			mCurrentState = RobotState.CLIMB;
			mTimeStartHatchPrescore = 0;
			mHasStartedHatchPrescore = false;
		}
		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)){
			mCurrentState = RobotState.WAITING_TO_LOAD;
			mTimeStartHatchPrescore = 0;
			mHasStartedHatchPrescore = false;
		}

	}

	/**
	 * 
	 */
	private void ballPrescore() {

		swerveDrive();

		mIntake.holdingBall();

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_CARGO)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_CARGO)) {
			mCurrentState = RobotState.BALL_FRONT_SCORE;
		}

		else if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_BALL_ROCKET)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.SCORE_PANEL_ROCKET)) {
			mCurrentState = RobotState.BALL_BACK_SCORE;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
			mCurrentState = RobotState.CLIMB;

		}
		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}

	
	private void ballFrontScore() {

		swerveDrive();

		if (mIntake.scoreBallHigh()
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_SCORED_BALL)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_SCORED_PANEL)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}

		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)) {
			mCurrentState = RobotState.CLIMB;
		}
		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}

	private void ballBackScore() {
		swerveDrive();
		// if robot or driver says scoring is done...
		if (mIntake.scoreBallLow()
				|| (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_SCORED_BALL)
				|| mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.HAS_SCORED_PANEL))) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB)){
			mCurrentState = RobotState.CLIMB;
		}
		if(mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.LEADSCREW_OVERRIDE)){
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}
	}

	private void defense() {
		// Do some action... move to a different state?
	}

	private void climb() {
		if (mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB_ABORT)) {
			mCurrentState = RobotState.WAITING_TO_LOAD;
		}

		if (!mJoystick.getRawButton(JoystickConstants.FinalRobotButtons.CLIMB_AUTO_MANUAL_SWITCH)) {
			mClimber.manualClimb();
		}
		else {
			mClimber.autoClimb();
		}
	}

	private void defaultState() {

	}

	public void disabled() {

		long timeDisabledStarted = System.currentTimeMillis();
		boolean ended = false;
		//createHeaderString();

		while (this.isDisabled()) {
			long timeElapsed = System.currentTimeMillis() - timeDisabledStarted;

			if (timeElapsed > 3000 && !ended) {
				if (RunConstants.SECONDARY_JOYSTICK) {
					if (!mInGame) {
						SmartDashboard.putString("JoystickCommand", "StartUpdating");
					} 
					else {
						SmartDashboard.putString("JoystickCommand", "StopUpdating");
					}
				}

				endGame();
				ended = true;
				//SmartDashboard.putString("CURRENT ROBOT MODE: ", "DISABLED");
			}

			SmartDashboard.putString("LEDCommand", DriverStation.getInstance().getAlliance().toString());

			if (mClimbJoystick.getTriggerPressed()) {
				// rotate autonomous routines to select which one to start with:
				if (mAutonomousRoutine == AutonomousRoutineType.DEFAULT) {
					mAutonomousRoutine = AutonomousRoutineType.DO_NOTHING;
				} 
				else if (mAutonomousRoutine == AutonomousRoutineType.DO_NOTHING) {
					mAutonomousRoutine = AutonomousRoutineType.DEFAULT;
				}
			}

			if(mClimbJoystick.getRawButtonPressed(2)){
				mNavX.setAngleAdjustment(0);
			}
			else if(mClimbJoystick.getRawButtonPressed(7)){
				mNavX.setAngleAdjustment(90);
			}
			else if(mClimbJoystick.getRawButtonPressed(10)){
				mNavX.setAngleAdjustment(-90);
			}
		}

		SmartDashboard.putString("AUTO ROUTINE:", mAutonomousRoutine.toString());
		Timer.delay(0.005); // wait for a motor update time
	}

	// ***************//
	// INITIALIZATION //
	// ***************//
	private void driveInit() {
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
			} 
			else {
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
			mDrive[i] = new CANSparkMax(drivePort, MotorType.kBrushless);
			mDrive[i].setInverted(driveReversed);
			mDrive[i].setIdleMode(IdleMode.kBrake);
			mDrive[i].setCANTimeout(10);
			mDrive[i].setOpenLoopRampRate(0.35);

			mDriveEncoders[i] = new CANEncoder(mDrive[i]);

			// initialize turn motors' encoders, as well as wheels:
			mEncoder[i] = new TalonAbsoluteEncoder(mTurn[i], ResourceFunctions.tickToAngle(turnOffset));
			mWheel[i] = new Wheel(mTurn[i], mDrive[i], mEncoder[i]);
		}

		mRobotAngle = new RobotAngle(mNavX, false, 0);
		mDriveTrain = new DriveTrain(mWheel, mController, mRobotAngle, mJoystick);
	}

	private void hatchIntakeInit() {
		mHatchRotaryPiston = new SingleSolenoidReal(Ports.ActualRobot.HATCH_ROTARY_SOLENOID_IN);
		mHatchLinearPiston = new SingleSolenoidReal(Ports.ActualRobot.HATCH_LINEAR_SOLENOID_IN);

		mHatchIntake = new HatchIntake(mHatchRotaryPiston, mHatchLinearPiston, mJoystick);
	}

	private void ballInit() {

		mBallRotary = new SingleSolenoidReal(Ports.ActualRobot.BALL_ROTARY);
		mBallLock = new SingleSolenoidReal(Ports.ActualRobot.BALL_LOCK);
		mBallRetain = new SingleSolenoidReal(Ports.ActualRobot.BALL_RETAIN);
		mUltraRight = new UltrasonicSensor(Ports.ActualRobot.ULTRASONIC_SENSOR_RIGHT);
		mUltraLeft = new UltrasonicSensor(Ports.ActualRobot.ULTRASONIC_SENSOR_LEFT);

		mBallIntake = new BallIntake(mBallRotary, mBallLock, mBallRetain, mUltraRight, mUltraLeft);
	}

	private void cameraInit() {

		mHatchCamera = new Limelight();
		// mHatchCamera.setStreamSecondary();
		mHatchCamera.setStreamMain();
		mHatchCamera.setVisionProcessor();
		mHatchCamera.setPipeline(CameraConstants.PIPELINE);
		mHatchCamera.setLedFromPipeline();

		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		//camera.setResolution(480, 360);
		//camera.setFPS(60);
		camera.setVideoMode(new VideoMode(VideoMode.PixelFormat.kMJPEG, 240, 180, 30));
		//camera.setBrightness(45);
	}

	private void leadscrewInit() {
		mLeadscrewTalon = new WPI_TalonSRX(Ports.ActualRobot.LEADSCREW);

		mLeadscrewTalon.setInverted(LeadscrewConstants.REVERSED);
		mLeadscrewTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		mLeadscrewTalon.setSensorPhase(LeadscrewConstants.ENCODER_REVERSED);
		mLeadscrewTalon.setNeutralMode(NeutralMode.Brake);
		mLeadscrewTalon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
		mLeadscrewTalon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyClosed, 10);
		mLeadscrewTalon.configClearPositionOnLimitR(false, 10);
		mLeadscrewTalon.configClearPositionOnLimitF(false, 10);
		// mLeadscrewTalon.configAllSettings(new TalonSRXConfiguration());

		mLeadscrewTalon.configForwardSoftLimitEnable(false, 10);
		// mLeadscrewTalon.configForwardSoftLimitThreshold(LeadscrewConstants.FORWARD_SOFT_LIMIT,
		// 10);
		mLeadscrewTalon.configReverseSoftLimitEnable(false, 10);
		// mLeadscrewTalon.configReverseSoftLimitThreshold(LeadscrewConstants.REVERSE_SOFT_LIMIT,
		// 10);

		mLeadscrewTalon.config_kP(0, LeadscrewConstants.PID.LEADSCREW_P, 10);
		mLeadscrewTalon.config_kI(0, LeadscrewConstants.PID.LEADSCREW_I, 10);
		mLeadscrewTalon.config_kD(0, LeadscrewConstants.PID.LEADSCREW_D, 10);
		mLeadscrewTalon.config_IntegralZone(0, LeadscrewConstants.PID.LEADSCREW_IZONE, 10);
		mLeadscrewTalon.configAllowableClosedloopError(0, LeadscrewConstants.PID.LEADSCREW_TOLERANCE, 10);

		mLeadscrewEncoder = new LeadscrewEncoder(mLeadscrewTalon);

		mLeadscrew = new Leadscrew(mLeadscrewTalon, mLeadscrewEncoder, mHatchCamera, mJoystick, mDriveTrain);
		
	}

	private void intakeInit() {
		mIntake = new Intake(mHatchIntake, mBallIntake, mLeadscrew, mHatchCamera, mDriveTrain, mJoystick);
	}

	private void climberInit() {
		mFrontClimbPiston = new DoubleSolenoidReal(Ports.ActualRobot.CLIMB_FRONT_PISTON_IN,
				Ports.ActualRobot.CLIMB_FRONT_PISTON_OUT, 1);
		mBackClimbPiston = new DoubleSolenoidReal(Ports.ActualRobot.CLIMB_BACK_PISTON_IN,
				Ports.ActualRobot.CLIMB_BACK_PISTON_OUT, 1);
		mFrontBreak = new DoubleSolenoidReal(Ports.ActualRobot.CLIMB_FRONT_PISTON_BREAK,
				Ports.ActualRobot.CLIMB_FRONT_PISTON_THROUGH, 1);
		mBackBreak = new DoubleSolenoidReal(Ports.ActualRobot.CLIMB_BACK_PISTON_BREAK,
				Ports.ActualRobot.CLIMB_BACK_PISTON_THROUGH, 1);

		mDriveClimbTalon0 = new WPI_TalonSRX(Ports.ActualRobot.CLIMB_DRIVE0);
		mDriveClimbTalon0.setInverted(ClimberConstants.DRIVE0_REVERSED);
		mDriveClimbTalon0.setNeutralMode(NeutralMode.Brake);

		mDriveClimbTalon1 = new WPI_TalonSRX(Ports.ActualRobot.CLIMB_DRIVE1);
		mDriveClimbTalon1.setInverted(ClimberConstants.DRIVE1_REVERSED);
		mDriveClimbTalon1.setNeutralMode(NeutralMode.Brake);

		mClimber = new ClimberPiston(mBackClimbPiston, mFrontClimbPiston, mDriveClimbTalon0, mDriveClimbTalon1,
				mDriveTrain, mClimbJoystick, mBackBreak, mFrontBreak, mNavX);
	}

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

		// SmartDashboard.putBoolean("Game Has Started: ", mInGame);
		// SmartDashboard.putNumber("Time Game Started: ", mGameStartMillis);
		SmartDashboard.putNumber("Time Elapsed: ", timeElapsed);

		StringBuilder logString = new StringBuilder();

		// for now it is one frame per line
		addLogValueInt(logString, (int) timeElapsed);


		if (RunConstants.RUNNING_DRIVE) {
			for (int i = 0; i < 4; i++) {
				addLogValueDouble(logString, mTurn[i].getOutputCurrent());
				addLogValueDouble(logString, mTurn[i].getMotorOutputVoltage());

				addLogValueDouble(logString, mDrive[i].getOutputCurrent());
				addLogValueDouble(logString, mDrive[i].getBusVoltage());

				addLogValueDouble(logString, mEncoder[i].getAngleDegrees());

			}

			addLogValueDouble(logString, mRobotAngle.getAngleDegrees());
		}

		if (RunConstants.RUNNING_PNEUMATICS) {
			addLogValueDouble(logString, mCompressor.getCompressorCurrent());
		}

		if (RunConstants.RUNNING_CLIMBER){
			addLogValueDouble(logString, mDriveClimbTalon0.getMotorOutputVoltage());
			addLogValueDouble(logString, mDriveClimbTalon0.getOutputCurrent());
			addLogValueDouble(logString, mDriveClimbTalon1.getMotorOutputVoltage());
			addLogValueDouble(logString, mDriveClimbTalon1.getOutputCurrent());
		}

		if(RunConstants.RUNNING_LEADSCREW){
			addLogValueDouble(logString, mLeadscrewTalon.getMotorOutputVoltage());
			addLogValueDouble(logString, mLeadscrewTalon.getOutputCurrent());
		}

		if(RunConstants.RUNNING_BALL){
			addLogValueBoolean(logString, mBallIntake.isHoldingBall());
		}

		addLogValueString(logString, mCurrentState.toString());

		SmartDashboard.putString("LogString", logString.toString());
	}

	public void createHeaderString(){
		StringBuilder headerString = new StringBuilder();

		String[] headers = { "Time left", 
				"Turn 1 Output Current", "Turn 1 Output Voltage", "Drive 1 Output Current", "Drive 1 Bus Voltage", "Turn 1 Angle",
				"Turn 2 Output Current", "Turn 2 Output Voltage", "Drive 2 Output Current", "Drive 2 Bus Voltage", "Turn 2 Angle",
				"Turn 3 Output Current", "Turn 3 Output Voltage", "Drive 3 Output Current", "Drive 3 Bus Voltage", "Turn 3 Angle",
				"Turn 4 Output Current", "Turn 4 Output Voltage", "Drive 4 Output Current", "Drive 4 Bus Voltage", "Turn 4 Angle",
				"Robot Angle", "Compressor Current", "Drive Climb 0 Output Voltage", "Drive Climb 0 Output Current",
				"Drive Climb 1 Output Voltage", "Drive Climb 1 Output Current",
				"Leadscrew Motor Output Voltage", "Leadscrew Motor Output Current", "Holding Ball", "Robot State" };


			for(int i = 0; i < headers.length - 1; i++){
				addLogValueString(headerString, headers[i]);
			}
			addLogValueEndString(headerString, headers[headers.length - 1]);

			SmartDashboard.putString("HeaderString", headerString.toString());
	}
}
