/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchIntakeConstants {

    public static class Piston {

        public static final Value OPEN = Value.kForward, CLOSE = Value.kReverse; //Need to check

    }

    public static class LeadScrew {

        public static final double PITCH = 12.0;

        public static final int OFFSET = 0;
	
        public static final boolean	ENCODER_REVERSED = true, REVERSED = false;
        
        public static class PID {

            public static final double 
                LEADSCREW_P = 0.0, 
                LEADSCREW_I = 0.0,
                LEADSCREW_D = 0.0;
            
            public static final int 
                LEADSCREW_TOLERANCE = 0,
                LEADSCREW_IZONE = 0;

        }

    }

}
