package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;

import java.util.concurrent.locks.ReentrantLock;


public class Odometry {
    private static boolean debug = false;
    private static final boolean USE_IMU = true;  // angle estimation is by IMU not by left/right encoder
    public static final double  COUNTS_PER_MOTOR_REV    = 2048 ; //8192   ;//  8192 for REV encoder  2048 -- for gobilda encoder
    public static final double  WHEEL_DIAMETER_M   = 0.048 * 1.02; //0.048 for gobilda tracking wheel, 0.072; //0.072;//0.096;     // for actually odometry tracking wheel
    public static final double  ENC_COUNTS_TO_M         =  (WHEEL_DIAMETER_M * Math.PI) / (COUNTS_PER_MOTOR_REV );  // for tracking wheel no * DRIVE_GEAR_REDUCTION
    public static final double  BASE_WIDTH_M = 0.306;          // left-right wheels distance, need test
    public static final double  MIDDLE_CENTER_DIS_M = 0.016 ;//0.016;// -0.045    0.23???   back or front  tracking wheel to robot rotation center 0.056, "+" for front, "-" for back
    public static final double MIDDLE_WHEEL_ADJ = 1.0;   //1.0, 0.97

    //make sure positive reading when robot move forward and move to left
    public static final int LEFT_ENCODER_DIR = -1;
    public static final int RIGHT_ENCODER_DIR = -1;
    public static final int MIDDLE_ENCODER_DIR = 1;

    private static final int PERIOD = 10 ;     //10ms


    // for thread control
    private static volatile double  lastIMUReadingAngle ;
    private static volatile double imuAngle = 0;



    private static volatile boolean taskRunFlag = false;

    private static volatile RobotPosition robotPos = new RobotPosition(0,0,0);
    private static volatile RobotVel robotLocalVel;

    private static volatile double wheelEstimateAngle = 0;

    public static volatile DcMotorEx leftEnc = null;
    public static volatile DcMotorEx rightEnc = null;
    public static volatile DcMotorEx middleEnc = null;

    //IMU
    public static volatile IMU imu;

    private static volatile boolean runFlag = false;

    private static volatile LinearOpMode mainTask = null;
    private static volatile ChassisSystem chassisSystem = null;
    private static volatile EstimationThread estimationThread = null;
    private static volatile Telemetry telemetry;
    private static volatile IMUUpdateThread imuUpdateThread = null;

    private static ReentrantLock odomPosLock = new ReentrantLock();
    private static ReentrantLock odomVelLock = new ReentrantLock();
    private static ReentrantLock imuThreadLock = new ReentrantLock();

    public static double middleR = 0;


    public Odometry(LinearOpMode mainTask, ChassisSystem chassis, RobotPosition pos, boolean auto){
        this.mainTask = mainTask;
        this.chassisSystem = chassis;
        this.robotPos.setPosition(pos);   // set starting the robot position


        this.robotLocalVel = new RobotVel();
        this.telemetry = this.mainTask.telemetry;
        //now using the general IMU initialize
        imu = mainTask.hardwareMap.get(IMU.class, "imu");


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // Note: if you choose two conflicting directions, this initialization will cause a code exception.


        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Mode", "calibrating Imu...");
        telemetry.update();
        mainTask.sleep(1000);

        if (auto){
            imu.resetYaw();
        }


        //map for odometry, please check the wire connection
        leftEnc = chassis.leftRearDrive;//hwMap.get(DcMotorEx.class, "LF");;
        rightEnc = chassis.rightFrontDrive;//hwMap.get(DcMotorEx.class, "RF");
        middleEnc = chassis.leftFrontDrive;//hwMap.get(DcMotorEx.class, "LR");


        // set current angle first
        imuAngle = robotPos.angle;   //IMU just reset, should be zero
        lastIMUReadingAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);



        estimationThread = new EstimationThread();
        //start the robot position
        estimationThread.start();


        if (!USE_IMU){
            //update by another thread
            imuUpdateThread = new IMUUpdateThread();
            imuUpdateThread.start();
        }






    }

    public RobotPosition getRobotPosition(){
        odomPosLock.lock();
        try{
            return robotPos;
        }finally {
            odomPosLock.unlock();
        }

    }

    public static double getAngle()
    {
        return robotPos.angle;
    }
    public static double getPosX(){return robotPos.x;}
    public static double getPosY(){return robotPos.y;}


    public RobotVel getRobotLocalVel(){
        odomVelLock.lock();
        try{
            return this.robotLocalVel;
        }finally {
            odomVelLock.unlock();
        }

    }

    public void setRobotPosition(RobotPosition pos){
        odomPosLock.lock();
        try{
            this.robotPos.setPosition(pos.x, pos.y,pos.angle);
        }finally {
            odomPosLock.unlock();
        }


    }

    //only update the robot x, y, call by April tag detection, angle, update by Gyro,
    public void setRobotPositionXY(RobotPosition pos){
        odomPosLock.lock();
        try{
            this.robotPos.x = pos.x;
            this.robotPos.y = pos.y;
            // not update angle
        }finally {
            odomPosLock.unlock();
        }


    }

    // only reset robot angle
    public void setRobotPositionAngle(double angleDegree){
        odomPosLock.lock();
        try{
           //only update the angle
            this.robotPos.angle = angleDegree;

        }finally {
            odomPosLock.unlock();
        }
    }


    /**
     *  will translate the IMU reading from -180 ~ 180 to absolute angle
     * @return absolute angle reading from IMU
     */

    public static double getHeadingAngle(){
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();
        double deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastIMUReadingAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        if (!Double.isNaN(deltaAngle)){
            imuAngle += deltaAngle;
            lastIMUReadingAngle = angles.getYaw(AngleUnit.DEGREES);
        }



        return imuAngle;
    }


    public double getIMUReading(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        return angles.getYaw(AngleUnit.DEGREES);
    }

    private static class IMUUpdateThread extends Thread {

        private static volatile boolean startFlag = false;  // accelerate distance

        public IMUUpdateThread() {
            startFlag = false;
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {
            while(!mainTask.isStopRequested()){
                if (mainTask.opModeIsActive()){
                    imuThreadLock.lock();
                    try{
                        imuAngle = getHeadingAngle();  // this thread will run around 6ms period
                    }finally {
                        imuThreadLock.unlock();
                    }

                    startFlag = true;
                }
                else if(startFlag){
                    break;
                }
            }

        }
    }

    public void updateRobotAngleByIMU(){
        if (!USE_IMU){
            imuThreadLock.lock();
            try{
                robotPos.angle = imuAngle;  //both are degree
            }finally {
                imuThreadLock.unlock();
            }


        }

    }

    public double getIMUAngle(){
        imuThreadLock.lock();
        try{
            return imuAngle;
        }finally {
            imuThreadLock.unlock();
        }

    }



    private static class EstimationThread extends Thread
    {
        private static volatile double leftLastPos = 0, rightLastPos = 0, middleLastPos = 0, lastIMUAngle = 0;

        private static volatile double accelerateDis = 0;  // accelerate distance
        public EstimationThread()
        {
            leftLastPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
            rightLastPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
            middleLastPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();
            lastIMUAngle = getHeadingAngle();       //read back current IMU reading
            runFlag = true;
            middleR = 0;
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            double leftEncPos, rightEncPos, middleEncPos;  // now the robot sensor reading

            double deltaL, deltaR, deltaMiddle, deltaTheta, deltaThetaIMU;

            double deltaTime = 0;
            double v1,v2;
            double last_time = 0;
            double angleLast = 0;
            double robotAngle= 0;

            double deltaXLocal, deltaYLocal;

            leftLastPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
            rightLastPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
            middleLastPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();
            imuAngle = getHeadingAngle();       //read back current IMU reading
            lastIMUAngle = imuAngle;
            angleLast = robotPos.angle;


            ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            last_time = runtime.milliseconds();
            runFlag = true;
            int readCnt = 0;
            runtime.reset();
            while(!mainTask.isStopRequested() && !isInterrupted() && runFlag)
            {
                try {
                    leftEncPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
                    rightEncPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
                    middleEncPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();

                    deltaL = (leftEncPos - leftLastPos) * ENC_COUNTS_TO_M * 1.0; //translate to meter distance
                    deltaR = (rightEncPos - rightLastPos) * ENC_COUNTS_TO_M * 1.0;  //translate to meter distance

                    //based on encoder reading, (deltaR - deltaL)  -- turn to Left are positive, counter-clockwise is positive, so we use deltaR - deltaL
                    //original using encoder to estimate the angle

                    deltaTheta = (deltaR - deltaL) / BASE_WIDTH_M; // radian,



                    if (USE_IMU){
                        imuAngle = getHeadingAngle();  //not using now, the imuHeading will update in another thread, not read IMU by I2C could save around (9ms - 2.5ms = 6.5ms)
                        deltaThetaIMU = Math.toRadians(imuAngle - lastIMUAngle);  // translate to radian
                        lastIMUAngle = imuAngle;
                    }
                   //
                    //middle back or front tracking wheel encoder
                    deltaMiddle = (middleEncPos - middleLastPos) * ENC_COUNTS_TO_M * MIDDLE_WHEEL_ADJ;   //1.0

                    double middleDis = middleEncPos * ENC_COUNTS_TO_M * 1.0;

                    if (Math.abs(robotPos.angle) > 5){
                        middleR = middleDis / Math.toRadians(robotPos.angle);
                    }



                    deltaXLocal = 0.5 * (deltaL + deltaR ) ;

                    deltaYLocal =  deltaMiddle - deltaTheta * MIDDLE_CENTER_DIS_M;

                    //use faster encoder reading to update
                    if (Math.abs(deltaTheta) > 0.000001)  //(Math.abs(deltaTheta) > 0.000001)  Math.abs(deltaTheta) > 2.0
                    {
                        double a = Math.sin(deltaTheta) / deltaTheta;
                        double b = (Math.cos(deltaTheta)-1) / deltaTheta;

                        v1 = a * deltaXLocal + b * deltaYLocal;
                        v2=  -b * deltaXLocal + a * deltaYLocal;
                    } else {
                        v1 = deltaXLocal;
                        v2 = deltaYLocal;
                    }

                    //using  encoder reading
                    robotAngle = Math.toRadians(robotPos.angle) + deltaTheta;  // to radian, here we use the encoder to calculate the deltaTheta
                    //robotAngle = Math.toRadians(robotPos.angle) + deltaThetaIMU;

                    double deltaXGlobal = Math.cos(robotAngle) * v1 - Math.sin(robotAngle) * v2;
                    double deltaYGlobal = Math.sin(robotAngle) * v1 + Math.cos(robotAngle) * v2;

                    odomPosLock.lock();
                    try{
                        if (!Double.isNaN(deltaXGlobal) && ! Double.isNaN(deltaYGlobal)) {
                            robotPos.x += deltaXGlobal;
                            robotPos.y += deltaYGlobal;
                        }


                        if (USE_IMU){
                            if (!Double.isNaN(deltaThetaIMU)) {
                                robotPos.angle += Math.toDegrees(deltaThetaIMU);  // long term, using IMU sensor
                            }
                        }
                        else{

                            //use encoder
                            if (!Double.isNaN(deltaTheta)) {
                                robotPos.angle += Math.toDegrees(deltaTheta);   // based on left-right wheel encoder
                            }
                        }
                    }finally {
                        odomPosLock.unlock();
                    }

                    /*
                    telemetry.addData("IMU ", "%3.2f", imuHeading);
                    telemetry.addData("Last IMU", "%3.2f", lastIMUAngle);

                    telemetry.update();*/




                    /******************    To numerically calculate the velocity of the robot ***************/
                    double samplePeriod = runtime.milliseconds() - last_time;
                    last_time += samplePeriod;

                    odomVelLock.lock();
                    try{
                        if (samplePeriod != 0) {
                            robotLocalVel.velX = deltaXLocal / samplePeriod * 1000.0;
                            robotLocalVel.velX = Math.abs(robotLocalVel.velX) > MecanumDriveLib.MAX_VEL ? Math.signum(robotLocalVel.velX) * MecanumDriveLib.MAX_VEL : robotLocalVel.velX;
                            robotLocalVel.velY = deltaYLocal / samplePeriod * 1000.0;
                            robotLocalVel.velY = Math.abs(robotLocalVel.velY) > MecanumDriveLib.MAX_VEL ? Math.signum(robotLocalVel.velY) * MecanumDriveLib.MAX_VEL : robotLocalVel.velY;


                            robotLocalVel.velAngle = 0.8 * robotLocalVel.velAngle + 0.2 * (robotPos.angle - angleLast) / samplePeriod * 1000.0;
                            angleLast = robotPos.angle;
                        }
                    }finally {
                        odomVelLock.unlock();
                    }





                    //update sensor reading here, do not read sensor directly again
                   // lastIMUAngle = imuHeading; // already is new data
                    leftLastPos = leftEncPos;
                    rightLastPos = rightEncPos;
                    middleLastPos = middleEncPos;
                    Thread.sleep(PERIOD);
                    /*
                    // just for debug the loop time to compare reading IMU and bulk reading for encoder
                    // with encoder bulk reading
                    // with IMU reading, the period is around 9.0ms
                    // without IMU reading, the period is around 2.5ms
                    readCnt++;
                    if (readCnt == 100){
                        telemetry.addData("time = ", "%f", runtime.milliseconds() / 100);
                        telemetry.update();
                        runtime.reset();
                        readCnt = 0;
                    }*/


                }
                // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
                // or by the interrupted exception thrown from the sleep function.
                //catch (InterruptedException e) {goStraightEndFlag = true;}
                // an error occurred in the run loop.
                catch (Exception e) {}
            }
            runFlag = false;
        }
    }

    public void stopThread()
    {
        runFlag = false;
    }

    public void modifyY(double deltaY){
        odomPosLock.lock();
        try{
            //only modify the Y
            this.robotPos.y += deltaY;

        }finally {
            odomPosLock.unlock();
        }
    }


}
