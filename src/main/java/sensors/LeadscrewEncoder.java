/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package sensors;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import constants.HatchIntakeConstants;

public class LeadscrewEncoder {

    private WPI_TalonSRX mTalon;
    private int mOffset;

    public LeadscrewEncoder(WPI_TalonSRX pTalon, int pOffset){
        mTalon = pTalon;
        mOffset = pOffset;
    }

    public int getOffset(){
        return mOffset;
    }
    
    // **********//
    // RESOURCE //
    // **********//

    public static double leadscrewTickToInch(double pTick) {
        return 1 / (HatchIntakeConstants.LeadScrew.PITCH * 4096) * pTick;
    }

    public static double leadscrewInchToTick(double pInch) {
        return HatchIntakeConstants.LeadScrew.PITCH * 4096 * pInch;
    }

    // ***********//
    // MEASUREMENT //
    // ***********//

    public int getRawTicks(){
        return mTalon.getSelectedSensorPosition(0);
    }

    public int getTicksFromEnd(){
        return getRawTicks() - mOffset;
    }

    public double getDistanceInInchesFromEnd(){
        return leadscrewTickToInch(getTicksFromEnd());
    }

}
