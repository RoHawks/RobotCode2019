/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package constants;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClimberConstants {

    public static final boolean
        DRIVE_REVERSED = false, // positive is forward
        FRONT_REVERSED = true, // negative brings legs up
        BACK_REVERSED = true; // negative brings legs up

    public static final double 
        SPEEDY_CLIMBER_LEGS_UP_SPEED = -0.6,
        SPEEDY_CLIMBER_LEGS_DOWN_SPEED = 0.8,
        SLOW_CLIMBER_LEGS_UP_SPEED = -0.4,
        SLOW_CLIMBER_LEGS_DOWN_SPEED = 0.6,
        CLIMBER_DRIVE_SPEED = 0.6; 

    public static final double
        SPEEDY_LEGS_UP_TIME = 8000,
        SPEEDY_LEGS_DOWN_TIME = 8000;

    public static final double
        HUNDRED_FROM_TOP = 560,
        NINETY_FROM_TOP = 0.9 * HUNDRED_FROM_TOP,
        TEN_FROM_TOP = 0.1 * HUNDRED_FROM_TOP;
    
    public static final Value 
        ENABLED = Value.kForward,
        DISABLED = Value.kReverse;

}
