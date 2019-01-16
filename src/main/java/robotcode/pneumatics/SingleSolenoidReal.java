package robotcode.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Solenoid;

public class SingleSolenoidReal {
	private Solenoid mSingleSolenoid;

	public SingleSolenoidReal(int pPort) {
		mSingleSolenoid = new Solenoid(pPort);
	}

	public void set(Value pDirection) {
		// forward maps to true, backward maps to false
		mSingleSolenoid.set(pDirection == Value.kForward);
	}

	public Value get() {
		return mSingleSolenoid.get() ? Value.kForward : Value.kReverse;
	}

	public void setOpposite() {
		this.set(this.get().equals(Value.kForward) ? Value.kReverse : Value.kForward);
	}

}
