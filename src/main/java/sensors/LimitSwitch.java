/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package sensors;

import edu.wpi.first.wpilibj.DigitalGlitchFilter;
import edu.wpi.first.wpilibj.DigitalInput;

/**
 * Add your docs here.
 */
public class LimitSwitch extends DigitalInput implements IDigitalInput{

    private DigitalGlitchFilter mFilter;
    private long mPeriod;

    public LimitSwitch(int pPort, long pNanoseconds){
        super(pPort);
        mPeriod = pNanoseconds;
        mFilter = new DigitalGlitchFilter();
        mFilter.add(this);
        mFilter.setPeriodNanoSeconds(mPeriod);
    }

    public boolean get() {
        return super.get();
    }

    public void setPeriod(long pNanoseconds){
        if (pNanoseconds != mPeriod){
            mFilter.setPeriodNanoSeconds(pNanoseconds);
            mPeriod = pNanoseconds;
        }
    }

    public long getPeriod(){
        return mPeriod;
    }
}
