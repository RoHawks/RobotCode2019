package robotcode;

import java.util.Arrays;

import constants.JoystickConstants;
import constants.RunConstants;
import edu.wpi.first.wpilibj.Joystick;

public class LocalJoystickTest extends Joystick {

    private int mNumButtons;

    private boolean[] mButtonsReleased;
    private boolean[] mButtonsPressed;

    private long[] mTimesReleased;
    private long[] mTimesPressed;

    public LocalJoystickTest(int port, int numButtons) {
        super(port);
        mNumButtons = numButtons;

        mButtonsReleased = new boolean[mNumButtons];
        mButtonsPressed = new boolean[mNumButtons];
        Arrays.fill(mButtonsReleased, false);
        Arrays.fill(mButtonsPressed, false);

        mTimesReleased = new long[mNumButtons];
        mTimesPressed = new long[mNumButtons];
        Arrays.fill(mTimesReleased, 0);
        Arrays.fill(mTimesPressed, 0);
    }

    public void periodicUpdate() {
    }

    public boolean getRawButtonReleased(int pButton) {
        return false;
    }

    public boolean getRawButtonPressed(int pButton) {
        return false;
    }

    public boolean getRawButton(int pButton) {
        return false;
    }

    public double getX(int pProfile) {
        if (!RunConstants.SECONDARY_JOYSTICK) { // if we're using the logitech attack 3
            return 0;
        } else { // if we're using the box
            return super.getX(); // just return the value
        }
    }

    public double getY(int pProfile) {
        if (!RunConstants.SECONDARY_JOYSTICK) { // if we're using the logitech attack 3
            return 0;
        } else { // if we're using the box
            return super.getY(); // just return the value
        }
    }

    public double getZ(int pProfile) {
        if (!RunConstants.SECONDARY_JOYSTICK) { // if we're using the logitech attack 3
            return 0;
        } else { // if we're using the box
            return super.getZ(); // just return the value
        }
    }

    public void updateProfile() {
        if (!RunConstants.SECONDARY_JOYSTICK) { // only update profiles if using the logitech attack 3
        }
    }

    public boolean[] getButtonsReleasedArray() {
        return mButtonsReleased;
    }

    public boolean[] getButtonsPressedArray() {
        return mButtonsPressed;
    }

    public void reset() {
    }

    public void resetReleased() {
    }

    public void resetPressed() {
    }
}
