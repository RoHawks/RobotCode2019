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
        DRIVE_REVERSED = false,
        FRONT_REVERSED = false,
        BACK_REVERSED = true;

    public static final double ClimberSpeed = 0.8;

    public static final Value ENABLED = Value.kReverse;

}
