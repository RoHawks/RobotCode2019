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
public class LateralSlider extends Mechanism{

    private State<LateralSlider> HATCH_INTAKE= new State<LateralSlider>(0, "HATCH INTAKE");
    private State<LateralSlider>[] LateralSliderStates;

    @Override
    public State<Mechanism>[] getStates() {
        return null;
    }

    @Override
    public void transitionTo(State<Mechanism> pState) {

    }
}
