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

    public LimitSwitch(int pPort){
        super(pPort);
        mFilter = new DigitalGlitchFilter();
    }

    public boolean get() {

        return super.get();
    }
}
