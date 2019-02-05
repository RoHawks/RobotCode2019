/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.pneumatics;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public interface SolenoidInterface {

    public void set (Value pDirection);

    public Value get();

    public void setOpposite();
    
}
