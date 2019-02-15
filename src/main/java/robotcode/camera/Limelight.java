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

public class Limelight {

    private static final double FOV = 59.6; // degrees

    public Limelight() {

    }


    // ******** //
    // RESOURCE //
    // ******** //
    public double getDoubleFromNetworkTable(String key) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key).getDouble(0);
    }

    public NetworkTableEntry getEntryFromNetworkTable(String key) {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry(key);
    }


    // **** //
    // DATA //
    // **** //
    public double xAngleToDistance() {
        return (CameraConstants.HEIGHT * Math.tan(Math.toRadians(getDoubleFromNetworkTable("tx"))));
    }

    public double yAngleToDistance() {
        return (CameraConstants.HEIGHT * Math.tan(Math.toRadians(getDoubleFromNetworkTable("ty"))));
    }

    public boolean hasTarget() {
        return getDoubleFromNetworkTable("tv") == 1;
    }

    public double getTargetAreaPercent() {
        return getDoubleFromNetworkTable("ta");
    }

    public double getTargetSkew() { //-90 to 0 degrees
        return getDoubleFromNetworkTable("ts");
    }

    public double getTargetWidth() { //in pixels
        return getDoubleFromNetworkTable("thor");
    }

    public double getTargetLength() {
        return getDoubleFromNetworkTable("tvert");
    }


    // ************ //
    // LED SETTINGS //
    // ************ //
    public void setLedFromPipeline(){
        getEntryFromNetworkTable("ledMode").setNumber(0);
    }

    public void setLedOff(){
        getEntryFromNetworkTable("ledMode").setNumber(1);
    }

    public void setLedBlink(){
        getEntryFromNetworkTable("ledMode").setNumber(2);
    }

    public void setLedOn(){
        getEntryFromNetworkTable("ledMode").setNumber(3);
    }


    // *********** //
    // CAMERA MODE //
    // *********** //
    public void setVisionProcessor(){
        getEntryFromNetworkTable("camMode").setNumber(0);
    }

    public void setDriverCamera(){
        getEntryFromNetworkTable("camMode").setNumber(1);
    }


    // ******** //
    // PIPELINE //
    // ******** //
    public void setPipeline(int pPipeline){
        getEntryFromNetworkTable("pipeline").setNumber(pPipeline);
    }

    public double getPipeline(){
        return getDoubleFromNetworkTable("getpipe");
    }

    public double getPipelineLatency() {
        return getDoubleFromNetworkTable("tl");
    }


    // ****** //
    // STREAM //
    // ****** //
    public void setStreamStandard(){
        getEntryFromNetworkTable("stream").setNumber(0);
    }
    
    public void setStreamMain(){
        getEntryFromNetworkTable("stream").setNumber(1);
    }
    
    public void setStreamSecondary(){
        getEntryFromNetworkTable("stream").setNumber(2);
        NetworkTableEntry e = getEntryFromNetworkTable("stream");
        System.out.print(e.getDouble(-1));
    }


    // ******** //
    // SNAPSHOT //
    // ******** //
    public void stopSnapshot(){
        getEntryFromNetworkTable("snapshot").setNumber(0);
    }

    public void startSnapshot(){
        getEntryFromNetworkTable("snapshot").setNumber(1);
    }

}
