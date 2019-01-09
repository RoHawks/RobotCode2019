package sensors;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class TalonRelativeEncoder extends RelativeEncoder {
	
	private WPI_TalonSRX mTalon;
	private boolean mReversed;

	public TalonRelativeEncoder(WPI_TalonSRX pTalon, boolean pReversed, double pTicksToRPM) 
	{
		super(pTicksToRPM);
		mTalon = pTalon;
		mReversed = pReversed;

		mTalon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
		mTalon.setInverted(mReversed);
	}

	public double getRawTicksPerSecond() 
	{
		return mTalon.getSelectedSensorVelocity(0);
	}
}
