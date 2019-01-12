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
		if(RunConstants.RUNNING_PNEUMATICS){
			mSingleSolenoid.set(pDirection == Value.kForward);
		}
	}

	public Value get() {
		if(RunConstants.RUNNING_PNEUMATICS){
			return mSingleSolenoid.get() ? Value.kForward : Value.kReverse;
		}
		else{
			return Value.kOff;
		}
	}

	public void setOpposite() {
		if(RunConstants.RUNNING_PNEUMATICS){
			this.set(this.get().equals(Value.kForward) ? Value.kReverse : Value.kForward);
		}
	}

}
