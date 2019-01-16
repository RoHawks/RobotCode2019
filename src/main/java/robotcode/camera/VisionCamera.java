/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.camera;

public abstract class VisionCamera {
  
    private Double mFOV = null;

    public abstract double xAngleToDistance(double angle);
    public abstract double yAngleToDistance(double angle);

}
