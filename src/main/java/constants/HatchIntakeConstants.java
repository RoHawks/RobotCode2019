/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchIntakeConstants {

    public static class RotaryPiston {

        public static final Value 
            EXPANDED = Value.kForward,
            CONTRACTED = Value.kReverse;

        public static final long
            EXPAND_TIME = 500,
            CONTRACT_TIME = 500;
    }

    public static class LinearPiston {
        public static final Value
            OUT = Value.kForward, 
            IN = Value.kReverse;

        public static final long
            IN_TIME = 500,
            OUT_TIME = 500;
    }

}
