/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.camera;

import constants.CameraConstants;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight extends VisionCamera{

    private double mFOV = 59.6; //degrees

    public Limelight(){

    }

    public double getDoubleFromNetworkTable(String key) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).getDouble(0);
    }

    public NetworkTableEntry getEntryFromNetworkTable (String key){
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key);
    }

    public double xAngleToDistance(double angle){
        return (CameraConstants.Limelight.HEIGHT * Math.tan(Math.toRadians(getDoubleFromNetworkTable("tx"))));
    }

    public double yAngleToDistance(double angle){
        return (CameraConstants.Limelight.HEIGHT * Math.tan(Math.toRadians(getDoubleFromNetworkTable("ty"))));
    }


}
