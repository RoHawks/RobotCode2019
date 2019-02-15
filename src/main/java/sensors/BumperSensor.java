/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package sensors;

import constants.Ports;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class BumperSensor {

    private DigitalInput mFrontLimitLeft, mFrontLimitRight;

    public enum BumperState {
        BOTH, LEFT, RIGHT, NEITHER
    }

    public BumperSensor(DigitalInput pFrontLimitLeft, DigitalInput pFrontLimitRight) {
        mFrontLimitLeft = pFrontLimitLeft;
        mFrontLimitRight = pFrontLimitRight;
    }

    public BumperState getState(){
        if(!mFrontLimitLeft.get() && !mFrontLimitRight.get()){
            return BumperState.BOTH;
        }
        else if(!mFrontLimitLeft.get()){
            return BumperState.LEFT;
        }
        else if(!mFrontLimitRight.get()){
            return BumperState.RIGHT;
        }
        SmartDashboard.putBoolean("bumper value", !mFrontLimitLeft.get() && !mFrontLimitRight.get());
        return BumperState.NEITHER;
        
    }

}
