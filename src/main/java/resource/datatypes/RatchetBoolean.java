/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package resource.datatypes;

/**
 * A boolean that only changes value once - once it negates, can't go back
 */
public class RatchetBoolean {

    private boolean mInitialValue;
    private boolean mValue;

    public RatchetBoolean(boolean pValue) {
        mValue = pValue;
        mInitialValue = pValue;
    }

    public void set(boolean pNew) {
        if (pNew != mInitialValue) {
            mValue = pNew;
        }
    }

    public boolean get() {
        return mValue;
    }

}
