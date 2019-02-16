/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode;

import constants.JoystickConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Joystick;

/**
 * Add your docs here.
 */
public class LocalJoystick extends Joystick {

    private int mProfiles = JoystickConstants.NUM_PROFILES;
    private int mCurrentProfile = 0;

    public LocalJoystick(int port) {
        super(port);
    }

    /**
     * Whether the button was released since the last check. Button indexes begin at 1.
     * @param pButton The button index, beginning at 1.
     * @return Whether the button was released since the last check.
     */
    public boolean getRawButtonReleased(int pButton) {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // if we're using the logitech attack 3
            int realButton = pButton - mCurrentProfile * 10;    // do all this stuff
            if (realButton <= 1 || realButton > 11) {
                return false;
            }
            return super.getRawButtonReleased(realButton);
        } else {                                                // if we're using the box
            if (pButton < 1 || pButton > 19) {                  // do this simple stuff
                return false;
            }
            return super.getRawButtonReleased(pButton);
        }
    }


    /**
     * Whether the buttno was released 
     * @param pButton
     * @return
     */
    public boolean getRawButtonReleaseTimed(int pButton){
        return false;
    }


    /**
     * Whether the button was pressed since the last check. Button indexes begin at 1.
     * @param pButton The button index, beginning at 1.
     * @return Whether the button was pressed since the last check.
     */
    public boolean getRawButtonPressed(int pButton) {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // if we're using the logitech attack 3
            int realButton = pButton - mCurrentProfile * 10;    // do all this stuff
            if (realButton <= 1 || realButton > 11) {
                return false;
            }
            return super.getRawButtonPressed(realButton);
        } else {                                                // if we're using the box
            if (pButton < 1 || pButton > 19) {                  // do this simple stuff
                return false;
            }
            return super.getRawButtonPressed(pButton);
        }
    }

    public boolean getRawButton(int pButton) {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // if we're using the logitech attack 3
            int realButton = pButton - mCurrentProfile * 10;    // do all this stuff
            if (realButton <= 1 || realButton > 11) {
                return false;
            }
            return super.getRawButton(realButton);
        } else {                                                // if we're using the box
            if (pButton < 1 || pButton > 19) {                  // do this simple stuff
                return false;
            }
            return super.getRawButton(pButton);
        }
    }

    public double getX(int pProfile) {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // if we're using the logitech attack 3
            if (pProfile != mCurrentProfile) {                  // only return the value if you're on the right profile
                return 0;
            } else {
                return super.getX();
            }
        } else {                                                // if we're using the box
            return super.getX();                                // just return the value
        }
    }

    public double getY(int pProfile) {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // if we're using the logitech attack 3
            if (pProfile != mCurrentProfile) {                  // only return the value if you're on the right profile
                return 0;
            } else {
                return super.getY();
            }
        } else {                                                // if we're using the box
            return super.getY();                                // just return the value
        }
    }

    public double getZ(int pProfile) {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // if we're using the logitech attack 3
            if (pProfile != mCurrentProfile) {                  // only return the value if you're on the right profile
                return 0;
            } else {
                return super.getZ();
            }
        } else {                                                // if we're using the box
            return super.getZ();                                // just return the value
        } 
    }

    public void updateProfile() {
        if (!RunConstants.SECONDARY_JOYSTICK) {                 // only update profiles if using the logitech attack 3
            if (super.getRawButtonReleased(1)) {
                mCurrentProfile = (mCurrentProfile + 1) % mProfiles;
            }
        }
    }

    public int getProfile() {
        return mCurrentProfile;
    }

}
