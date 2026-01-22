package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import static com.qualcomm.hardware.rev.RevHubOrientationOnRobot.xyzOrientation;




public class Odometry {

    private static final boolean USE_IMU = false;  // angle estimation is by IMU not by left/right encoder
    public static final double  COUNTS_PER_MOTOR_REV    = 8192   ;//  8192 for REV encoder
    public static final double  WHEEL_DIAMETER_M   = 0.072; //0.072;//0.096;     // for actually odometry tracking wheel
    public static final double  ENC_COUNTS_TO_M         =  (WHEEL_DIAMETER_M * Math.PI) / (COUNTS_PER_MOTOR_REV );  // for tracking wheel no * DRIVE_GEAR_REDUCTION
    public static final double  BASE_WIDTH_M = 0.272;          // left-right wheels distance, need test
    public static final double  MIDDLE_CENTER_DIS_M = 0.153;// 0.23???   back or front  tracking wheel to robot rotation center 0.056, "+" for front, "-" for back

    //make sure positive reading when robot move forward and move to left
    public static final int LEFT_ENCODER_DIR = 1;
    public static final int RIGHT_ENCODER_DIR = 1;
    public static final int MIDDLE_ENCODER_DIR = -1;

    private static final int PERIOD = 10 ;     //10ms


    // for thread control
    private static volatile YawPitchRollAngles  lastAngles ;
    private static volatile double imuAngle = 0;


    private static volatile boolean taskRunFlag = false;
    private static volatile double lastIMUAngle = 0;

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

    public Odometry(LinearOpMode mainTask, HardwareMap hwMap, ChassisSystem chassis, RobotPosition pos, Telemetry telemetry, boolean auto){
        this.mainTask = mainTask;
        this.chassisSystem = chassis;
        this.robotPos = pos;   // set starting the robot position
        this.robotLocalVel = new RobotVel();
        this.telemetry = telemetry;
        //now using the general IMU initialize
        imu = hwMap.get(IMU.class, "imu");

        // The next three lines define the desired axis rotations.
        // To Do: EDIT these values to match YOUR mounting configuration. how to configure it please follow the sample code "SensorIMUNonOrthogonal.java"
        double xRotation = 0;  // enter the desired X rotation angle here. USB port up 90
        double yRotation = -90;  // enter the desired Y rotation angle here. logo point out to right . left, -90
        double zRotation = 0;  // enter the desired Z rotation angle here. no rotation
        Orientation hubRotation = xyzOrientation(xRotation, yRotation, zRotation);

        // Now initialize the IMU with this mounting orientation
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(hubRotation);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        telemetry.addData("Mode", "calibrating Imu...");
        telemetry.update();

        imu.resetYaw();

        //map for odometry, please check the wire connection
        leftEnc = chassis.leftFrontDrive;//hwMap.get(DcMotorEx.class, "LF");;
        rightEnc = chassis.rightFrontDrive;//hwMap.get(DcMotorEx.class, "RF");
        middleEnc = chassis.leftRearDrive;//hwMap.get(DcMotorEx.class, "LR");

        // set current angle first
        imuAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);   //IMU just reset, should be zero
        lastAngles = imu.getRobotYawPitchRollAngles();
        lastIMUAngle = getHeadingAngle();

        estimationThread = new EstimationThread();


        //start the robot position
        estimationThread.start();


    }

    public static RobotPosition getRobotPosition(){
        return robotPos;
    }

    public static double getAngle()
    {
        return robotPos.angle;
    }
    public static double getPosX(){return robotPos.x;}
    public static double getPosY(){return robotPos.y;}


    public RobotVel getRobotLocalVel(){
        return this.robotLocalVel;
    }

    public void setRobotPosition(RobotPosition pos){
        this.robotPos = pos;
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
        double deltaAngle = angles.getYaw(AngleUnit.DEGREES) - lastAngles.getYaw(AngleUnit.DEGREES);

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        imuAngle += deltaAngle;
        lastAngles = angles;
        return imuAngle;
    }
    public double getIMUReading(){
        YawPitchRollAngles angles = imu.getRobotYawPitchRollAngles();

        return angles.getYaw(AngleUnit.DEGREES);
    }


    private static class EstimationThread extends Thread
    {
        private static volatile double leftLastPos = 0, rightLastPos = 0, backLastPos = 0, lastIMUAngle = 0;



        private static volatile double accelerateDis = 0;  // accelerate distance
        public EstimationThread()
        {
            leftLastPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
            rightLastPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
            backLastPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();
            lastIMUAngle = getHeadingAngle();       //read back current IMU reading
            runFlag = true;
        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            double leftEncPos, rightEncPos, backEncPos, imuHeading;  // now the robot sensor reading
            double deltaL, deltaR, deltaMiddle, deltaTheta, deltaThetaIMU;

            double deltaTime = 0;
            double v1,v2;
            double last_time = 0;
            double angleLast = 0;
            double robotAngle= 0;

            double deltaXLocal, deltaYLocal;

            leftLastPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
            rightLastPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
            backLastPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();
            imuHeading = getHeadingAngle();       //read back current IMU reading
            lastIMUAngle = imuHeading;
            angleLast = robotPos.angle;


            ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            last_time = runtime.milliseconds();
            runFlag = true;
            while(!mainTask.isStopRequested() && !isInterrupted() && runFlag)
            {
                try {
                    leftEncPos = LEFT_ENCODER_DIR * leftEnc.getCurrentPosition();
                    rightEncPos = RIGHT_ENCODER_DIR * rightEnc.getCurrentPosition();
                    backEncPos = MIDDLE_ENCODER_DIR * middleEnc.getCurrentPosition();


                    imuHeading = getHeadingAngle();
                    deltaL = (leftEncPos - leftLastPos) * ENC_COUNTS_TO_M * 1.0; //translate to meter distance
                    deltaR = (rightEncPos - rightLastPos) * ENC_COUNTS_TO_M * 1.0;  //translate to meter distance

                    //based on encoder reading, (deltaR - deltaL)  -- turn to Left are positive, counter-clockwise is positive, so we use deltaR - deltaL
                    //original using encoder to estimate the angle

                    deltaTheta = (deltaR - deltaL) / BASE_WIDTH_M; // radian,

                        //if using gyro(IMU)
                    // deltaTheta = Math.toRadians(imuHeading - lastIMUAngle);  // translate to radian

                    deltaThetaIMU = Math.toRadians(imuHeading - lastIMUAngle);  // translate to radian
                    //middle back or front tracking wheel encoder
                    deltaMiddle = (backEncPos - backLastPos) * ENC_COUNTS_TO_M * 1.0;


                    deltaXLocal = 0.5 * (deltaL + deltaR ) ;

                    deltaYLocal =  deltaMiddle - deltaTheta * MIDDLE_CENTER_DIS_M;

                    if (Math.abs(deltaTheta) > 0.000001)  //(Math.abs(deltaTheta) > 0.000001)  Math.abs(deltaTheta) > 2.0
                    {
                        double a = Math.sin(deltaTheta)/deltaTheta;
                        double b = (Math.cos(deltaTheta)-1)/deltaTheta;

                        v1 = a * deltaXLocal + b * deltaYLocal;
                        v2=  -b * deltaXLocal + a * deltaYLocal;
                    } else {
                        v1 = deltaXLocal;
                        v2 = deltaYLocal;
                    }

                    robotAngle = Math.toRadians(robotPos.angle) + deltaTheta;  // to radian
                    //Using IMU instead estimated angle
                    double deltaXGlobal = Math.cos(robotAngle) * v1 - Math.sin(robotAngle) * v2;
                    double deltaYGlobal = Math.sin(robotAngle) * v1 + Math.cos(robotAngle) * v2;
                    robotPos.x += deltaXGlobal;
                    robotPos.y += deltaYGlobal;
                    robotPos.angle += Math.toDegrees(deltaThetaIMU);  // long term, using IMU sensor
                    //robotPos.angle += Math.toDegrees(deltaTheta);   // based on left-right wheel encoder



                    /******************    To numerically calculate the velocity of the robot ***************/
                    double samplePeriod = runtime.milliseconds() - last_time;
                    last_time += samplePeriod;

                    if (samplePeriod != 0) {
                        robotLocalVel.velX = deltaXLocal / samplePeriod * 1000.0;
                        robotLocalVel.velY = deltaYLocal / samplePeriod * 1000.0;


                        robotLocalVel.velAngle = 0.8 * robotLocalVel.velAngle + 0.2 * (robotPos.angle - angleLast) / samplePeriod * 1000.0;
                        angleLast = robotPos.angle;
                    }



                    //update sensor reading here, do not read sensor directly again
                    lastIMUAngle = imuHeading; // already is new data
                    leftLastPos = leftEncPos;
                    rightLastPos = rightEncPos;
                    backLastPos = backEncPos;
                    Thread.sleep(PERIOD);

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



}
