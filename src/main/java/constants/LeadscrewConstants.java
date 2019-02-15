/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package constants;

/**
 * Add your docs here.
 */
public class LeadscrewConstants {
    public static final double 
        PITCH = 12.0, 
        LENGTH = 7, //14.875, 
        MIDDLE = LENGTH / 2,
        SOFT_LIMIT = 0.75, 
        LEADSCREW_CAMERA_TOLERANCE = 0.2; // inches

    public static final boolean 
        ENCODER_REVERSED = true, 
        REVERSED = true;

    public static final int
        FORWARD_SOFT_LIMIT = 400000,
        REVERSE_SOFT_LIMIT = 1000;

    public static class PID {

        public static final double 
            LEADSCREW_P = 0.1, 
            LEADSCREW_I = 0.0007, // 0.0003
            LEADSCREW_D = 0.0001;

        public static final int 
            LEADSCREW_TOLERANCE = 3000, 
            LEADSCREW_IZONE = 30000;
    }
}
