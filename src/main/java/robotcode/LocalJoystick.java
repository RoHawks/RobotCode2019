/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode;

import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class LocalJoystick extends Joystick{

    private int mProfiles = 3;
    private int mCurrentProfile = 0;

    public LocalJoystick(int port){
        super(port);
    }

    public boolean getRawButtonReleased(int pButton) {
        int realButton = pButton - mCurrentProfile * 10;
        if(realButton <= 1 || realButton > 11){
            return false;
        }
        return super.getRawButtonReleased(realButton);
    }

    public boolean getRawButtonPressed(int pButton){
        int realButton = pButton - mCurrentProfile * 10;
        if(realButton <= 1 || realButton > 11){
            return false;
        }
        return super.getRawButtonPressed(realButton);
    }

    public boolean getRawButton(int pButton){
        int realButton = pButton - mCurrentProfile * 10;
        if(realButton <= 1 || realButton > 11){
            return false;
        }
        return super.getRawButton(realButton);
    }

    public double getX(int pProfile) {
        if (pProfile != mCurrentProfile) {
            return 0;
        } else {
            return super.getX();
        }
    }

    public double getY(int pProfile) {
        if (pProfile != mCurrentProfile) {
            return 0;
        } else {
            return super.getY();
        }
    }

    public double getZ(int pProfile){
        if (pProfile != mCurrentProfile) {
            return 0;
        } else {
            return super.getZ();
        }
    }

    public void updateProfile() {
        if (super.getRawButtonReleased(1)){
            mCurrentProfile = (mCurrentProfile + 1) % mProfiles;
        }
    }

    public int getProfile() {
        return mCurrentProfile;
    }

}
