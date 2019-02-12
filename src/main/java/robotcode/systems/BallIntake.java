/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package robotcode.systems;

import constants.BallIntakeConstants;
import edu.wpi.first.wpilibj.DigitalInput;
import robotcode.pneumatics.SolenoidInterface;

public class BallIntake {

    private SolenoidInterface 
        mRotary,
        mLock, //front
        mRetain; //middle

    private DigitalInput mBreakbeam;

    public BallIntake (SolenoidInterface pRotary, SolenoidInterface pLock, SolenoidInterface pRetain){
        mRotary = pRotary;
        mLock = pLock;
        mRetain = pRetain;
    }

    private enum BallIntakeState{
        CARGOSHIP, ROCKET, HOLD, MANUAL
    }

    // **************//
    // ROTARY PISTON //
    // **************//

    public void forward(){
        mRotary.set(BallIntakeConstants.RotaryPiston.OPEN);
    }

    public void backward(){
        mRotary.set(BallIntakeConstants.RotaryPiston.CLOSE);
    }

    public void setRotaryOpposite(){
        mRotary.setOpposite();
    }

    // ************//
    // LINEAR LOCK //
    // ************//

    public void lock(){
        mLock.set(BallIntakeConstants.LinearLockPiston.OPEN);
    }

    public void letGo(){
        mLock.set(BallIntakeConstants.LinearLockPiston.CLOSE);
    }

    public void setLockOpposite(){
        mLock.setOpposite();
    }

    // **************//
    // LINEAR RETAIN //
    // **************//

    public void retain(){
        mRetain.set(BallIntakeConstants.LinearLockPiston.OPEN);
    }

    public void release(){
        mRetain.set(BallIntakeConstants.LinearLockPiston.CLOSE);
    }

    public void setRetainOpposite(){
        mRetain.setOpposite();
    }

    public boolean isHoldingBall(){
        return !mBreakbeam.get();
    } //TZ add delayed boolean
}
