/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package sensors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import constants.LeadscrewConstants;

public class LeadscrewEncoder {

    private WPI_TalonSRX mTalon;

    public LeadscrewEncoder(WPI_TalonSRX pTalon) {
        mTalon = pTalon;
    }

    // **********//
    // RESOURCE //
    // **********//

    /**
     * converts tick unit into inches
     * @param pTick length in ticks
     * @return the equivalent length in inches
     */
    public static double leadscrewTickToInch(double pTick) {
        return 1 / (LeadscrewConstants.PITCH * 4096) * pTick;
    }

    /**
     * converts inch lengths to ticks
     * @param pInch length in inches
     * @return equivalent length in ticks
     */
    public static double leadscrewInchToTick(double pInch) {
        return LeadscrewConstants.PITCH * 4096 * pInch;
    }


    // ***********//
    // MEASUREMENT //
    // ***********//

    /**
     * 
     * @return the value of the leadscrew encoder at the current position
     */
    public int getRawTicks() {
        return mTalon.getSelectedSensorPosition(0);
    }

    /**
     * 
     * @return the number of ticks from the end of the leadscrew (wherever it was zeroed)
     */
    public int getTicksFromEnd() {
        return getRawTicks();
    }

    /**
     * 
     * @return the distance from the end of the leadscrew (where it was zeroed) in inches
     */
    public double getDistanceInInchesFromEnd() {
        return leadscrewTickToInch(getTicksFromEnd());
    }

}
