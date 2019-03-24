/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode;

import java.util.Arrays;

import constants.JoystickConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Joystick;


public class LocalJoystick extends Joystick {

    private int mProfiles = JoystickConstants.NUM_PROFILES;
    private int mCurrentProfile = 0;

    private boolean[] mButtonsReleased = new boolean[32];
    private boolean[] mButtonsPressed = new boolean[32];

    private long[] mTimesReleased = new long[32];
    private long[] mTimesPressed = new long[32];

    public LocalJoystick(int port) {
        super(port);

        Arrays.fill(mButtonsReleased, false);
        Arrays.fill(mButtonsPressed, false);

        Arrays.fill(mTimesReleased, 0);
        Arrays.fill(mTimesPressed, 0);
    }

    public void periodicUpdate() {
        for (int i = 0; i < JoystickConstants.BUTTONS; i++) {

            boolean buttonReleased = getRawButtonReleased(i+1);
            if (buttonReleased) {
                mTimesReleased[i] = System.currentTimeMillis();
                mButtonsReleased[i] = true;
            }
            else {
                mButtonsReleased[i] = buttonReleased || System.currentTimeMillis() - mTimesReleased[i] < JoystickConstants.MILLISECONDS_RESET;
            }

            boolean buttonPressed = getRawButtonPressed(i+1);
            if (buttonPressed) {
                mTimesPressed[i] = System.currentTimeMillis();
                mButtonsPressed[i] = true;
            }
            else {
                mButtonsPressed[i] = buttonPressed || System.currentTimeMillis() - mTimesPressed[i] < JoystickConstants.MILLISECONDS_RESET;
            }
            // else if(mTimesReleased[i] > mTimesPressed[i]){
            //     mButtonsPressed[i] = buttonPressed || System.currentTimeMillis() - mTimesReleased[i] < JoystickConstants.MILLISECONDS_RESET;
            // }
        }
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
            // boolean toReturn = mButtonsReleased[realButton + 1];
            // mButtonsReleased[realButton + 1] = false;
            // mTimesReleased[realButton + 1] = 0;
            // return toReturn;
            return super.getRawButtonReleased(realButton);
        } 
        else {                                                // if we're using the box
            if (pButton < 1 || pButton > JoystickConstants.BUTTONS) {                  // do this simple stuff
                return false;
            }
            // boolean toReturn = mButtonsReleased[pButton + 1];
            // mButtonsReleased[pButton + 1] = false;
            // mTimesReleased[pButton + 1] = 0;
            // return toReturn;
            return super.getRawButtonReleased(pButton);
        }
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
            // boolean toReturn = mButtonsPressed[realButton + 1];
            // mButtonsPressed[realButton + 1] = false;
            // mTimesPressed[realButton + 1] = 0;
            // return toReturn;
            return super.getRawButtonPressed(realButton);
        } 
        else {                                                // if we're using the box
            if (pButton < 1 || pButton > JoystickConstants.BUTTONS) {                  // do this simple stuff
                return false;
            }
            // boolean toReturn = mButtonsPressed[pButton + 1];
            // mButtonsPressed[pButton + 1] = false;
            // mTimesPressed[pButton + 1] = 0;
            // return toReturn;
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
            if (pButton < 1 || pButton > JoystickConstants.BUTTONS) {                  // do this simple stuff
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

    public boolean[] getButtonsReleasedArray() {
        return mButtonsReleased;
    }

    public boolean[] getButtonsPressedArray() {
        return mButtonsPressed;
    }

    public int getProfile() {
        return mCurrentProfile;
    }

    public void reset(){
        
    }

    public void resetReleased(){

    }

    public void resetPressed(){

    }
}