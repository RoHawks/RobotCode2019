/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import robotcode.LocalJoystick;
import robotcode.pneumatics.SolenoidInterface;
import constants.HatchIntakeConstants;
import constants.JoystickConstants;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class HatchIntake {

    // **********//
    // VARIABLES //
    // **********//

    // joysticks used
    private LocalJoystick mJoystick;

    // pistons
    private SolenoidInterface 
        mRotaryPiston,
        mLinearPiston;

    // ***********//
    // INITIALIZE //
    // ***********//
    public HatchIntake(SolenoidInterface pRotaryPiston, SolenoidInterface pLinearPiston, LocalJoystick pJoystick) {
        mRotaryPiston = pRotaryPiston;
        mLinearPiston = pLinearPiston;

        mJoystick = pJoystick;
    }


    // **************//
    // ROTARY PISTON //
    // **************//
    public void expand() {
        mRotaryPiston.set(HatchIntakeConstants.RotaryPiston.OPEN);
    }

    public void contract() {
        mRotaryPiston.set(HatchIntakeConstants.RotaryPiston.CLOSE);
    }

    public void setRotaryOpposite() {
        mRotaryPiston.setOpposite();
    }

    // **************//
    // LINEAR PISTON //
    // **************//
    public void in() {
        mLinearPiston.set(HatchIntakeConstants.LinearPiston.CLOSE);
    }

    public void out() {
        mLinearPiston.set(HatchIntakeConstants.LinearPiston.OPEN);
    }

    public void setLinearOpposite() {
        mLinearPiston.setOpposite();
    }

    public void enactMovement() {
        // rotary piston
        if (mJoystick.getRawButtonReleased(JoystickConstants.HatchIntakeButtons.EXPAND_CONTRACT)) {
            mRotaryPiston.setOpposite();
        }

        // linear piston
        if (mJoystick.getRawButtonReleased(JoystickConstants.HatchIntakeButtons.IN_OUT)) {
            mLinearPiston.setOpposite();
        }
    }
}

