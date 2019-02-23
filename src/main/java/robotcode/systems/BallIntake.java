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
import sensors.UltrasonicSensor;

public class BallIntake {

    private SolenoidInterface 
        mRotary,
        mLock, //front
        mRetain; //middle

    private UltrasonicSensor mUltra;

    public BallIntake (SolenoidInterface pRotary, SolenoidInterface pLock, SolenoidInterface pRetain, UltrasonicSensor pSensor){
        mRotary = pRotary;
        mLock = pLock;
        mRetain = pRetain;
        mUltra = pSensor;
    }

    private enum BallIntakeState{
        CARGOSHIP, ROCKET, HOLD, MANUAL
    }

    // **************//
    // ROTARY PISTON //
    // **************//

    public void forward(){
        mRotary.set(BallIntakeConstants.RotaryPiston.FORWARD);
    }

    public void backward(){
        mRotary.set(BallIntakeConstants.RotaryPiston.BACKWARD);
    }

    public void setRotaryOpposite(){
        mRotary.setOpposite();
    }

    // ************//
    // LINEAR LOCK //
    // ************//

    public void lock(){
        mLock.set(BallIntakeConstants.LinearLockPiston.LOCK);
    }

    public void letGo(){
        mLock.set(BallIntakeConstants.LinearLockPiston.LET_GO);
    }

    public void setLockOpposite(){
        mLock.setOpposite();
    }

    // **************//
    // LINEAR RETAIN //
    // **************//

    public void retain(){
        mRetain.set(BallIntakeConstants.LinearRetainPiston.RETAIN);
    }

    public void release(){
        mRetain.set(BallIntakeConstants.LinearRetainPiston.RELEASE);
    }

    public void setRetainOpposite(){
        mRetain.setOpposite();
    }

    public boolean isHoldingBall(){
        return mUltra.getCooked();
    } //TZ add delayed boolean

    public UltrasonicSensor getUltrasonic(){
        return mUltra;
    }
}
