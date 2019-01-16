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

        public static final 
            Value OPEN = Value.kForward,
            CLOSE = Value.kReverse;

    }

    public static class LinearPiston {
        public static final Value
            OPEN = Value.kForward, 
            CLOSE = Value.kReverse;
    }

}
