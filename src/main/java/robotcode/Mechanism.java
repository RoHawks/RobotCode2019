/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode;

/**
 * Add your docs here.
 */
public abstract class Mechanism {

    public abstract State<Mechanism>[] getStates();

    public abstract void transitionTo(State<Mechanism> pState);

}
