/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import robotcode.pneumatics.DoubleSolenoidReal;
import constants.HatchIntakeConstants;
import constants.JoystickConstants;
import constants.RunConstants;

import edu.wpi.first.wpilibj.*;

public class HatchIntake {

    // **********//
    // VARIABLES //
    // **********//

    // joysticks used
    private Joystick mJoystick;

    // pistons
    private DoubleSolenoidReal 
        mRotaryPiston,
        mLinearPiston;

    // ***********//
    // INITIALIZE //
    // ***********//
    public HatchIntake(DoubleSolenoidReal pRotaryPiston, DoubleSolenoidReal pLinearPiston, Joystick pJoystick) {
        mRotaryPiston = pRotaryPiston;
        mLinearPiston = pLinearPiston;

        mJoystick = pJoystick;
    }

    // private enum HatchIntakeState {
    //     MANUAL, //combine these into intake class? name for intake class? states for hatchintake
    // }

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
            setRotaryOpposite();
        }

        // linear piston
        if (mJoystick.getRawButtonReleased(JoystickConstants.HatchIntakeButtons.IN_OUT)) {
            setLinearOpposite();
        }
    }
}

// Potentially add methods to check if hit the limit switches, if so, disable
// movement in one direction
// Have preset positions
