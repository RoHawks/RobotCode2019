package robotcode.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class DoubleSolenoidReal implements SolenoidInterface {
	private DoubleSolenoid mDoubleSolenoid;
	private Value mCurrent;

	public DoubleSolenoidReal(int pInPort, int pOutPort) {
		mDoubleSolenoid = new DoubleSolenoid(pInPort, pOutPort);
		mCurrent = this.getActual();
	}

	public DoubleSolenoidReal(int pInPort, int pOutPort, int pModuleNumber){
		mDoubleSolenoid = new DoubleSolenoid(pModuleNumber, pInPort, pOutPort);
		mCurrent = this.getActual();
	}

	public void set(Value pDirection) {
		if(pDirection != mCurrent){
			mDoubleSolenoid.set(pDirection);
			mCurrent = pDirection;
		}
	}

	public Value get(){
		return mCurrent;
	}

	public Value getActual() {
		return mDoubleSolenoid.get();
	}

	public void setOpposite() {
		this.set((this.get() == Value.kForward) ? Value.kReverse : Value.kForward);
	}

}
