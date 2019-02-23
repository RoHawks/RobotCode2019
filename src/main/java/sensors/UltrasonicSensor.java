    /*----------------------------------------------------------------------------*/
    /* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
    /* Open Source Software - may be modified and shared by FRC teams. The code   */
    /* must be accompanied by the FIRST BSD license file in the root directory of */
    /* the project.                                                               */
    /*----------------------------------------------------------------------------*/

    package sensors;

    import edu.wpi.first.wpilibj.AnalogInput;
    import constants.BallIntakeConstants;

    public class UltrasonicSensor extends AnalogInput {

        private long mTimeLastChanged;
        private boolean mActualValue;
        private boolean mReturnedValue;
        private boolean mKeepingTrackValue;

        public UltrasonicSensor(int pPort) {
            super(pPort);
            mActualValue = getBallUnfiltered();
            mReturnedValue = getBallUnfiltered();
            mKeepingTrackValue = mActualValue;
            mTimeLastChanged = System.currentTimeMillis();
        }

        public boolean getCooked() {
            mActualValue = this.getBallUnfiltered();

            if(mActualValue != mKeepingTrackValue){
                mKeepingTrackValue = mActualValue;
                mTimeLastChanged = System.currentTimeMillis();
            }

            if(mActualValue != mReturnedValue && System.currentTimeMillis() - mTimeLastChanged > BallIntakeConstants.Ultrasonic.BALL_TIME_THRESHOLD){
                mReturnedValue = mActualValue;
                mKeepingTrackValue = mActualValue;
            }

            return mReturnedValue;
        }

        private boolean getBallUnfiltered(){
            return this.getValue() > BallIntakeConstants.Ultrasonic.BALL_DISTANCE_THRESHOLD;
        }

    }
