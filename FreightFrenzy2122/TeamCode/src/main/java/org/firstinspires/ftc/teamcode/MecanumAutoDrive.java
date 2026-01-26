package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.os.SystemClock.sleep;
import static java.lang.Math.abs;
import static java.lang.Math.signum;


public class MecanumAutoDrive {
    static final double     COUNTS_PER_MOTOR_REV    = 383.6 ;    // eg: matrix motor + gobilda gearbox
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // mecanum drive chassis
    static final double     WHEEL_DIAMETER_M   = 0.10 ;     // For figuring circumference
    static final double     COUNTS_PER_M         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_M * 3.1415);
    static final double DIS_2WHEELS = 0.735;          // for 4 mecanum wheels, actually test value is 0.74 distance between 2 wheels, for encoder turning

    static final double MAX_MOVE_SPEED = 1.0;
    static final double MIN_MOVE_SPEED= 0.05;		// need test

    static final double STOP_DISTANCE =  1.0;       //full speed brake distance
    // which means the max speed to stop min speed in 0.5m
    static final double GO_SLOWDOWN_RATIO = 	(MAX_MOVE_SPEED - MIN_MOVE_SPEED) / STOP_DISTANCE;
    static final double ACCELERATION_DIS = 0.1;         // in 0.2m could be acceleration to 1.0
    static final double ACCELERATION__RATIO = (MAX_MOVE_SPEED - MIN_MOVE_SPEED) / ACCELERATION_DIS;
    static final double SLOW_DRIVE_DIS = 0.05;   //last 0.1m, drive at MIN_MOVE_SPEED;

    double goKp = 0.05;
    // for thread control
    volatile Orientation currentAngles;        //used for control
    volatile double yawReading ;
    volatile double lastYawReading;
    volatile double headingAngle ;

    volatile boolean goStraightEndFlag = true;
    MecanumAutoDrive.DriveStraightThread driveStraightThread = null;

    static final double START_SLOWTURN_ANGLE = 90;      //less than 90, start to slow down
    static final double TURN_MAX_POWER = 1.0    ;
    static final double TURN_MIN_POWER = 0.05;
    static final double SLOWTURN_ANGLE = 0;         //now set to "0" in 5 degre turn by TURN_MIN_POWER
    volatile  boolean turnEndFlag = true;

    MecanumAutoDrive.TurnAngleThread turnAngleThread = null;

    public enum TURN_METHOD{
        TWO_WHEEL,
        LEFT_WHEEL,
        RIGHT_WHEEL
    }

    MecanumChassis robot;
    private ElapsedTime runtime = new ElapsedTime();
    Telemetry telemetry = null;

    /* construction function, need pass the hardware robot , and telemetry
    /* parameter: hardware robot, and telemetry
    /* need set current angle and heading
    */
    public MecanumAutoDrive(MecanumChassis robot, Telemetry telemetry){
     this.robot = robot;
     this.runtime = new ElapsedTime();
     this.telemetry = telemetry;
    // set current angle first
    currentAngles   = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
    yawReading =  AngleUnit.DEGREES.fromUnit(currentAngles.angleUnit, currentAngles.firstAngle);
    lastYawReading = yawReading;
    headingAngle = yawReading;

    }



    /* this function will heading angle from -180 ~ + 180 original range

     */
    public double getHeadingAngle(){
        double heading = 0;
        currentAngles   = this.robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        yawReading = AngleUnit.DEGREES.fromUnit(currentAngles.angleUnit, currentAngles.firstAngle);

        double delta = yawReading - lastYawReading;
        if (abs(delta) >= 180){
            //it should be over +180 to -180 point
            if (lastYawReading < 0) {
                delta = yawReading - 180 - (180 + lastYawReading);
            }
            if (lastYawReading > 0){
                delta = 180 - lastYawReading + 180 + yawReading;
            }

        }
        headingAngle = headingAngle + delta;
        lastYawReading = yawReading;
        return headingAngle;
    }

    /*
    stop all the driving motors
     */
    public void stopRobot()
    {
        this.robot.rightFrontDrive.setPower(0);
        this.robot.leftFrontDrive.setPower(0);
        this.robot.rightRearDrive.setPower(0);
        this.robot.leftRearDrive.setPower(0);
    }

    /* set the robot command out*/
    public void driveRobot(double leftPower, double rightPower){
        leftPower    = Range.clip(leftPower, -1.0, 1.0) ;
        rightPower   = Range.clip(rightPower, -1.0, 1.0) ;
        this.robot.rightFrontDrive.setPower(rightPower);
        this.robot.rightRearDrive.setPower(rightPower);
        this.robot.leftRearDrive.setPower(leftPower);
        this.robot.leftFrontDrive.setPower(leftPower);
    }

    /* goStraightTask
    /* paramter:
    /*           dis     : unit:m, always positive
    /*           setAng  : unit: degree, absolute angle, not relative angle
    /*           power   : max go forward power, negative means move back
    /*           momentumDis:  unit:m
    /*           timeOuts: Time out unit: second
    /* in this function will start the actually drive thread
     */
    public void goStraightTask(double dis, double setAng, double power, double momentumDis, double timeOuts){
        runtime.reset();

        if (dis > momentumDis){
            dis = dis - momentumDis;
        }

        driveStraightThread = new DriveStraightThread(dis,setAng,power);

        driveStraightThread.start();
        //wait here
        while ((!goStraightEndFlag) && (runtime.seconds() < timeOuts)){
            //wait here
            this.telemetry.addData("Encoder",  "at %7d :%7d:%7d:%7d",
                    robot.leftFrontDrive.getCurrentPosition(),
                    robot.leftRearDrive.getCurrentPosition(),
                    robot.rightFrontDrive.getCurrentPosition(),
                    robot.rightRearDrive.getCurrentPosition());
            this.telemetry.update();
            Thread.yield();

        }
        //out
        driveStraightThread.interrupt();
        stopRobot();
        sleep(100);  //wait for a while between task to give some time to update the encoder reading
    }

    /*
    /* actually drive thread
    /*

     */
    private class DriveStraightThread extends Thread
    {
        private double distance = 0;
        private double setAngle = 0;
        private double setPower = 0;

        private double lfStartPos = 0, rfStartPos = 0, lrStartPos = 0, rrStartPos = 0;
        private double accelerateDis = 0;  // accelerate distance
        private double goSlowDownDis = 0;
        private double remainDis = 0;
        private double dynamicKp = 0;
        double disDone = 0;
        double goSpeed = 0;
        double leftCmd = 0;
        double rightCmd = 0;
        double ctrlValue = 0;
        //intialize the current encoder reading
        public DriveStraightThread(double dis, double setAng, double power)
        {
            this.distance = dis;
            this.setAngle = setAng;
            this.setPower = power;
            goStraightEndFlag = false;
            this.lfStartPos = robot.leftFrontDrive.getCurrentPosition();
            this.rfStartPos = robot.rightFrontDrive.getCurrentPosition();
            this.lrStartPos = robot.leftRearDrive.getCurrentPosition();
            this.rrStartPos = robot.rightRearDrive.getCurrentPosition();
            this.goSlowDownDis = (double)(abs(this.setPower) - MIN_MOVE_SPEED) / GO_SLOWDOWN_RATIO;
            this.accelerateDis = (double)(abs(this.setPower) - MIN_MOVE_SPEED) / ACCELERATION__RATIO;
            this.remainDis = 0;
            this.disDone = 0;
            this.goSpeed = 0;
            this.setName("DriveStraightThread");

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {


            try
            {
                while (!isInterrupted())
                {
                    //here we keep driving the robot to set distance with set angle
                    getHeadingAngle();    // get heading angle

                    disDone = (abs (robot.leftFrontDrive.getCurrentPosition() - lfStartPos) + abs(robot.rightFrontDrive.getCurrentPosition() - rfStartPos) +
                            + abs(robot.leftRearDrive.getCurrentPosition() - lrStartPos) + abs(robot.rightRearDrive.getCurrentPosition() - rrStartPos)) /4;
                    disDone = disDone/COUNTS_PER_M;
                    this.remainDis = this.distance - disDone;
                    if (disDone < this.distance) {
                        //keep driving
                        // if in acceleration distance and setPower > 0.5, we need increase the speed to slowly
                        if ((this.disDone < accelerateDis) && (abs(this.setPower) > 0.5) && (this.distance > accelerateDis)) {
                            goSpeed = signum(this.setPower) * MIN_MOVE_SPEED + signum(this.setPower) * (disDone) * ACCELERATION__RATIO;

                        } else {


                            if (this.remainDis < this.goSlowDownDis)   //start to slow down
                            {

                                //start to slow down
                                if (remainDis <= SLOW_DRIVE_DIS)        //last 0.1m, drive at MIN_MOVE_SPEED
                                {
                                    //if in slow drive distance, move at minimum speed
                                    goSpeed = signum(this.setPower) * MIN_MOVE_SPEED;
                                } else {

                                    goSpeed = signum(this.setPower) * MIN_MOVE_SPEED + signum(this.setPower) * (remainDis - SLOW_DRIVE_DIS) * GO_SLOWDOWN_RATIO;

                                    if (abs(goSpeed) < MIN_MOVE_SPEED) {
                                        goSpeed = signum(this.setPower) * MIN_MOVE_SPEED;
                                    }
                                }

                            } else {
                                goSpeed = this.setPower;
                            }
                        }


                        // gyro angle correction here
                        dynamicKp = (0.1 + 0.9 * abs(goSpeed)) * goKp;
                        ctrlValue = dynamicKp * (this.setAngle - headingAngle);

                        leftCmd = goSpeed - ctrlValue ;
                        rightCmd = goSpeed + ctrlValue ;

                        driveRobot(leftCmd, rightCmd);
                    }
                    else{
                        stopRobot();
                        goStraightEndFlag = true;

                    }

                    Thread.yield();
                }

            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {goStraightEndFlag = true;}

        }
    }


   /* turnRbotTask
    /* parameter:
    /*           setAng  : unit: degree, absolute angle, not relative angle
    /*           power   : max go forward power, negative means move back
    /*           deadZone: unit: degree, in deadZone, no more move, normally set as 1 degree
    /*           turnMethod:  2 wheel turn or left/right wheel turn
    /*           timeOuts: Time out unit: second
    /* in this function will start the actually drive thread
     */

    public void turnRobotTask(double setAng, double power, double deanZone, TURN_METHOD turnMethod,double timeOuts){
        runtime.reset();

        turnAngleThread = new TurnAngleThread(setAng,power,deanZone,turnMethod );

        turnAngleThread.start();
        while ((!turnEndFlag) && (runtime.seconds() < timeOuts)){

            telemetry.addData("Heading=","%.1f",headingAngle);
            telemetry.update();
            Thread.yield();

        }
        //out
        turnAngleThread.interrupt();
        stopRobot();
        sleep(100);   //wait for a while

    }



    private class TurnAngleThread extends Thread
    {

        private double setAngle = 0;
        private double setPower = 0;
        private double deadZone = 1.0;      // 1 degree
        double goSpeed = 0;
        double leftCmd = 0;
        double rightCmd = 0;
        double ctrlValue = 0;
        private double angleError = 0;
        private double slowRatio = 0;
        private TURN_METHOD turnMethod = TURN_METHOD.TWO_WHEEL;
        public TurnAngleThread(double setAng, double power, double deadZone, TURN_METHOD turnMethod)
        {
            this.setAngle = setAng;
            this.setPower = power;
            goStraightEndFlag = false;
            this.deadZone = deadZone;
            this.goSpeed = 0;
            this.setName("TurnThread");
            turnEndFlag = false;
            this.turnMethod = turnMethod;
            this.slowRatio = (this.setPower - TURN_MIN_POWER) / (START_SLOWTURN_ANGLE - SLOWTURN_ANGLE);

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {


            try
            {
                while (!isInterrupted())
                {
                    //here we keep driving the robot to set distance with set angle
                    getHeadingAngle();      // getHeading angle
                    angleError = this.setAngle - headingAngle;
                    if (abs(angleError) > deadZone){

                        if (abs(angleError) > START_SLOWTURN_ANGLE){
                            ctrlValue = signum(angleError) * this.setPower;
                        }
                        else if(abs(angleError) > SLOWTURN_ANGLE){

                            ctrlValue = signum(angleError) * (abs(angleError) - SLOWTURN_ANGLE) * slowRatio;
                        }
                        else{
                            ctrlValue = signum(angleError) * TURN_MIN_POWER;
                        }

                        if (abs(ctrlValue) < TURN_MIN_POWER) ctrlValue = signum(ctrlValue) * TURN_MIN_POWER;

                        if (this.turnMethod == TURN_METHOD.TWO_WHEEL) {
                            driveRobot(-ctrlValue, ctrlValue);
                        }
                        else if (this.turnMethod == TURN_METHOD.LEFT_WHEEL){
                            driveRobot(-ctrlValue * 2, 0);
                        }
                        else if (this.turnMethod == TURN_METHOD.RIGHT_WHEEL){
                            driveRobot(0,ctrlValue * 2);
                        }
                    }
                    else{
                        stopRobot();
                        turnEndFlag = true;

                    }

                    Thread.yield();
                }
            }
            // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
            // or by the interrupted exception thrown from the sleep function.
            //catch (InterruptedException e) {goStraightEndFlag = true;}
            // an error occurred in the run loop.
            catch (Exception e) {turnEndFlag = true;}

        }
    }


    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.  // not implemented
     */
    public void encoderDrive(double speed,
                             double leftM, double rightM,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;

        // Ensure that the opmode is still active
        if (true) {//opModeIsActive()

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = this.robot.leftFrontDrive.getCurrentPosition() + (int)(leftM * COUNTS_PER_M);
            newRightFrontTarget = this.robot.rightFrontDrive.getCurrentPosition() + (int)(rightM * COUNTS_PER_M);
            newLeftRearTarget = this.robot.leftRearDrive.getCurrentPosition() + (int)(leftM * COUNTS_PER_M);
            newRightRearTarget = this.robot.rightRearDrive.getCurrentPosition() + (int)(rightM * COUNTS_PER_M);
            this.robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            this.robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            this.robot.leftRearDrive.setTargetPosition(newLeftRearTarget);
            this.robot.rightRearDrive.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            this.robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            this.runtime.reset();
            this.robot.leftFrontDrive.setPower(Math.abs(speed));
            this.robot.rightFrontDrive.setPower(Math.abs(speed));
            this.robot.leftRearDrive.setPower(Math.abs(speed));
            this.robot.rightRearDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() ||
                            robot.leftRearDrive.isBusy() || robot.rightRearDrive.isBusy())) {
                telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                        this.robot.leftFrontDrive.getCurrentPosition(),
                        this.robot.leftRearDrive.getCurrentPosition(),
                        this.robot.rightFrontDrive.getCurrentPosition(),
                        this.robot.rightRearDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            this.robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(200);

        }
    }


    /* angle is relative with current angle, turn to left, positive, turn to right negative
    /* work not very precise
     *  Method to perfmorm a relative move , based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running. // not implemented
     */
    public void encoderTurn(double speed,
                             double angle, TURN_METHOD turn_method,
                             double timeoutS) {
        int newLeftFrontTarget;
        int newRightFrontTarget;
        int newLeftRearTarget;
        int newRightRearTarget;
        double leftM = 0,rightM = 0;

        // Ensure that the opmode is still active
        if (true) {//opModeIsActive()

            if (turn_method == TURN_METHOD.TWO_WHEEL){
                leftM = -angle / 360 * Math.PI * DIS_2WHEELS ;
                rightM = angle/ 360 * Math.PI * DIS_2WHEELS ;
            }
            else if(turn_method == TURN_METHOD.LEFT_WHEEL){
                leftM = -angle / 360 * 2 * Math.PI * DIS_2WHEELS;
                rightM = 0;
            }
            else if(turn_method == TURN_METHOD.RIGHT_WHEEL){
                leftM = 0;
                rightM = angle / 360 * 2 * Math.PI * DIS_2WHEELS ;
            }

            // Determine new target position, and pass to motor controller
            newLeftFrontTarget = this.robot.leftFrontDrive.getCurrentPosition() + (int)(leftM * COUNTS_PER_M);
            newRightFrontTarget = this.robot.rightFrontDrive.getCurrentPosition() + (int)(rightM * COUNTS_PER_M);
            newLeftRearTarget = this.robot.leftRearDrive.getCurrentPosition() + (int)(leftM * COUNTS_PER_M);
            newRightRearTarget = this.robot.rightRearDrive.getCurrentPosition() + (int)(rightM * COUNTS_PER_M);
            this.robot.leftFrontDrive.setTargetPosition(newLeftFrontTarget);
            this.robot.rightFrontDrive.setTargetPosition(newRightFrontTarget);
            this.robot.leftRearDrive.setTargetPosition(newLeftRearTarget);
            this.robot.rightRearDrive.setTargetPosition(newRightRearTarget);

            // Turn On RUN_TO_POSITION
            this.robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            this.robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            this.runtime.reset();
            this.robot.leftFrontDrive.setPower(Math.abs(speed));
            this.robot.rightFrontDrive.setPower(Math.abs(speed));
            this.robot.leftRearDrive.setPower(Math.abs(speed));
            this.robot.rightRearDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while ( (runtime.seconds() < timeoutS) &&
                    (robot.leftFrontDrive.isBusy() || robot.rightFrontDrive.isBusy() ||
                            robot.leftRearDrive.isBusy() || robot.rightRearDrive.isBusy())) {
                telemetry.addData("Path0",  "Starting at %7d :%7d:%7d:%7d",
                        this.robot.leftFrontDrive.getCurrentPosition(),
                        this.robot.leftRearDrive.getCurrentPosition(),
                        this.robot.rightFrontDrive.getCurrentPosition(),
                        this.robot.rightRearDrive.getCurrentPosition());

                telemetry.update();
            }

            // Stop all motion;
            stopRobot();

            // Turn off RUN_TO_POSITION
            this.robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.leftRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            this.robot.rightRearDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(200);

        }
    }



}
