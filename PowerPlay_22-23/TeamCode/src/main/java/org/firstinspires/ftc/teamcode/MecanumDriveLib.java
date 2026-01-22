package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Config
public class MecanumDriveLib {
    // here is the 1150 RPM motor, after gear is 575RPM  = 9.58RPS(per second). translate to wheel speed around 2.89m/s
    // wheel perimeter is 0.096 * pi = 0.3016
    // for 312 RPM motor 312 / 60 * 0.3016 = 1.56832

    private static final double MAX_VEL =  1.56832;  // 2.1865 * 0.8 =1.75 2.1865 * 0.9 =1.96785 m/s, some lose here, need test, For 435 RPM motor, 435 /60 * PI * WHEEL_D(0.096) = 2.1865 * 0.8 =1.75


    private static volatile LinearOpMode mainTask = null;

    private static volatile ElapsedTime runtime = new ElapsedTime();
    private static volatile Telemetry telemetry = null;
    private static volatile RobotPosition robotPos ;
    private static volatile Odometry odometry ;
    private static volatile ChassisSystem chassis;
    private static volatile RobotVel robotVel;
    private static volatile TurnCtrlThread turnCtrlThread = null;
    private static volatile P2PDiveThread p2PDiveThread = null;
    private static volatile HoldPositionThread holdPositionThread = null;
    private static volatile boolean holdPositionTaskFlag = false;

    private static volatile boolean turnTaskFlag = false;
    private static volatile boolean p2pTaskFlag = false;
    private static volatile boolean stopAtEnd = true;

    private final double accelRate = 0.05;        // will accelerate from 0 to 1.0(max) in 0.5S
    private int PERIOD = 10;    // 10ms

    // for dashboard to live modify the value, need set it as public and static, suggest use UPPER CASE name
    public static double p2pXKp = 2.0;
    public static double p2pXKd = 3.5;
    public static double p2pXKi = 0.08;


    public static double p2pYKp = 4.5;
    public static double p2pYKd = 8.0;
    public static double p2pYKi = 0.1;


    public static double p2pTurnKp = 0.01;
    public static double p2pTurnKi = 0.001;
    public static double p2pTurnKd = 0.05;


    public static double turnKp = 0.015;
    public static double turnKi = 0.01;
    public static double turnKd = 0.05;

    public static double turnKIMax = 0.2;

    public static double p2pXKIMax = 0.2;
    public static double p2pYKIMax = 0.4;

    public static double xVelKp = 0.5;
    public static double yVelKp = 0.5;

    public static double holdPosXKp = 0.2;
    public static double holdPosXKd = 2.0;
    public static double holdPosXKi = 0.05;
    public static double holdPosXMaxKi = 0.3;

    public static double holdPosYKp = 0.4;
    public static double holdPosYKd = 1.0;
    public static double holdPosYKi = 0.1;
    public static double holdPosYMaxKi = 0.5;

    public static double holdPosTurnKp = 0.01;
    public static double holdPosTurnKd = 0.1;
    public static double holdPosTurnKi = 0.05;
    public static double holdPosTurnMaxKi = 0.3;
    public static double rAlpha = 1.0;
    public static double powerXMin = 0.05;
    public static double powerYMin = 0.05;
    public static double powerRMin = 0.05;


    private static double ACCEL_TOFULL_TIME = 100;  //accelerate to full power time  500






    private static volatile double disError = 0;
    private static volatile double angleError = 0;
    private static volatile double powerP = 0;  // linear velocity
    public static volatile double powerR = 0;  // rotate velocity
    public static volatile double powerX = 0;
    public static volatile double powerY = 0;


    MecanumDriveLib(LinearOpMode mainTask,Odometry odom, ChassisSystem chassis, Telemetry telemetry)
    {
        this.mainTask = mainTask;
        this.odometry = odom;
        this.chassis = chassis;
        this.telemetry = telemetry;
        powerYMin = 0.1;
        powerXMin = 0.1;
        powerRMin = 0.1;

        setP2PPID(2.0,0.08,3.5,4.5,0.1,8.0,0.01,0.001,0.05);
    }

    public void stopAllThread()
    {
        if (turnCtrlThread != null){
            turnCtrlThread.interrupt();
            turnTaskFlag = false;
            turnCtrlThread = null;
        }

        if (p2PDiveThread != null)
        {
            p2PDiveThread.interrupt();
            p2pTaskFlag = false;
            p2PDiveThread = null;
        }

        if (holdPositionThread != null){
            holdPositionThread.interrupt();
            holdPositionTaskFlag = false;
            holdPositionThread = null;
        }
    }

    public void stopRobot()
    {
        stopAllThread();
        //no any slew rate control
        chassis.stopRobot();

    }


    public void turnToAngle(double setAngle, double tolerance, TURN_WHEEL turnWheel, double rVel, boolean stopAtEnd,int timeOut)
    {
        if (mainTask.opModeIsActive()) {
            turnTaskFlag = true;

            turnCtrlThread = new TurnCtrlThread(setAngle, tolerance, turnWheel, rVel, stopAtEnd, timeOut);

            turnCtrlThread.start();
        }

    }
    public void stopTurnToAngleTask(){
        turnTaskFlag = false;
        //mainTask.sleep(50); // give sometime the thread exit while loop
        if (turnCtrlThread != null){
            turnCtrlThread = null;
        }
    }

    public void turnToPoint(RobotPosition targetPos,  double tolerance ,TURN_WHEEL turnWh , double rVel , boolean stopAtEnd,int timeout )
    {
        if (mainTask.opModeIsActive()) {
            RobotPosition state = odometry.getRobotPosition();
            double pointAngle = Math.atan2(targetPos.y - state.y, targetPos.x - state.x);  // radian -pi +Pi
            double deltaAngle = Math.toDegrees(pointAngle) - angleWrap180(state.angle);
            double targetA = deltaAngle + state.angle;
            turnToAngle(targetA, tolerance, turnWh, rVel, stopAtEnd, timeout);
        }
    }

    public void setTurnPID(double kp, double ki, double kd)
    {
        this.turnKp = kp;
        this.turnKi = ki;
        this.turnKd = kd;
    }



    private static class TurnCtrlThread extends Thread {
        private static volatile double target = 0;
        private static volatile double tolerance = 0;
        private static volatile PosPIDController turnPID = null;
        private static volatile TURN_WHEEL turnWheel = TURN_WHEEL.WHEELS_TURN;
        private static volatile int timeOut;
        private static volatile boolean stopAtEnd = true;
        private static volatile double rVel = 0;
        private static volatile int period = 10;  // 10ms

        TurnCtrlThread(double target, double tolerance, TURN_WHEEL turnWheel,double rVel,  boolean stopAtEnd, int timeOut)
        {

            this.turnWheel = turnWheel;
            this.target = target;
            this.tolerance = tolerance;
            this.timeOut = timeOut;
            turnPID = new PosPIDController(turnKp,turnKi,turnKd);
            turnPID.setIntegralRange(15 * tolerance);
            turnPID.setMaxSumIntegral(turnKIMax);
            turnPID.setOutputRange(-1.0,1.0);
            turnPID.setFilerAlpha(rAlpha);
            this.stopAtEnd = stopAtEnd;
            this.rVel = rVel;
        }


        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run()
        {
            RobotPosition pos = odometry.getRobotPosition();
            runtime.reset();
            do {
                pos = odometry.getRobotPosition();
                angleError = target - pos.angle;  // in degree
                powerR = turnPID.calculate(angleError);

                powerR = Math.abs(powerR) > rVel? Math.signum(powerR) * rVel: powerR;

                if (turnWheel == TURN_WHEEL.WHEELS_TURN){
                    chassis.driveRobot(-powerR, -powerR, powerR,powerR);
                }
                else if(turnWheel == TURN_WHEEL.LEFT_WHEEL){
                    chassis.driveRobot(-powerR, -powerR, 0,0);
                }
                else if(turnWheel == TURN_WHEEL.RIGHT_WHEEL){
                    chassis.driveRobot(0, 0, powerR,powerR);
                }
                try {
                    Thread.sleep(period);
                }
                catch (Exception e)
                {

                }


            } while(!mainTask.isStopRequested() && !interrupted() && Math.abs(angleError) > Math.abs(tolerance) && turnTaskFlag && runtime.milliseconds() < timeOut);

            if (this.stopAtEnd){
                chassis.driveRobot(0,0,0,0);
            }
            turnTaskFlag = false;

        }

    }

    public boolean turnIsSettled(){
        return !turnTaskFlag;
    }

    public void setP2PPID(double p2pfKp, double p2pfKi, double p2pfKd,double p2psKp, double p2psKi, double p2psKd, double p2pTurnKp, double p2pTurnKi, double p2pTurnKd){
        this.p2pXKp = p2pfKp;
        this.p2pXKi = p2pfKi;
        this.p2pXKd = p2pfKd;
        this.p2pYKp = p2psKp;
        this.p2pYKi = p2psKi;
        this.p2pYKd = p2psKd;


        this.p2pTurnKp = p2pTurnKp;
        this.p2pTurnKi = p2pTurnKi;
        this.p2pTurnKd = p2pTurnKd;
    }
    public void setP2PPID(double p2pfKp, double p2pfKi, double p2pfKd){
        this.p2pXKp = p2pfKp;
        this.p2pXKi = p2pfKi;
        this.p2pXKd = p2pfKd;

    }

    public void setP2PTurnPID(double p2pTurnKp, double p2pTurnKi, double p2pTurnKd){

        this.p2pTurnKp = p2pTurnKp;
        this.p2pTurnKi = p2pTurnKi;
        this.p2pTurnKd = p2pTurnKd;
    }


    public void stopP2PTask()
    {
        p2pTaskFlag = false;
        mainTask.sleep(50); // give sometime the thread exit while loop
        if (p2PDiveThread != null){
            p2PDiveThread = null;
        }
    }
    public boolean p2pIsSettled()
    {
        return !p2pTaskFlag;
    }

    public void p2pDrive(RobotPosition targetPos, double p2pVel, double turnVel, double disTolerance, double angleTolerance, double endVel,boolean stopAtEnd, int timeOut)
    {
        if (mainTask.opModeIsActive()) {

            p2pTaskFlag = true;
            p2PDiveThread = new P2PDiveThread(targetPos, p2pVel, turnVel, disTolerance, angleTolerance, endVel, stopAtEnd, timeOut);

            p2PDiveThread.start();
        }
    }

    private static class P2PDiveThread extends Thread {
        //point to point drive, x,y and turn at same time
        private static volatile RobotPosition targetPos;
        private static volatile double disTolerance;
        private static volatile double angleTolerance;
        private static volatile boolean stopAtEnd = true;
        private static volatile int timeOut;
        private static volatile double endvel;
        private static volatile double p2pVel;
        private static volatile double turnVel;
        private static volatile PosPIDController rPID = null;
        private static volatile PosPIDController fPID = null;
        private static volatile PosPIDController sPID = null;

        private int period = 10;  // 10ms

        P2PDiveThread(RobotPosition targetPos, double p2pVel, double turnVel, double disTolerance, double angleToLerance, double endVel,boolean stopAtEnd, int timeOut)
        {
            this.disTolerance = disTolerance;
            this.angleTolerance = angleToLerance;
            this.stopAtEnd = stopAtEnd;
            this.timeOut = timeOut;
            this.endvel = endVel;
            this.p2pVel = p2pVel;
            this.turnVel = turnVel;
            this.targetPos = targetPos;

        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {

            rPID = new PosPIDController(p2pTurnKp, p2pTurnKi,p2pTurnKd);
            rPID.setOutputRange(-0.6,0.6);
            rPID.setIntegralRange(20);
            rPID.setMaxSumIntegral(turnKIMax);

            fPID = new PosPIDController(p2pXKp, p2pXKi,p2pXKd);
            fPID.setOutputRange(-1.0,1.0);
            fPID.setIntegralRange(0.10);
            fPID.setMaxSumIntegral(p2pXKIMax);

            sPID = new PosPIDController(p2pYKp, p2pYKi,p2pYKd);
            sPID.setOutputRange(-1.0,1.0);
            sPID.setIntegralRange(0.1);
            sPID.setMaxSumIntegral(p2pYKIMax);


            double theta = 0;


            double deltaX = 0, deltaY = 0;

            RobotVel robotVel = odometry.getRobotLocalVel(); // is in m/s unit

            RobotPosition nowPos = odometry.getRobotPosition();

            double powerX = 0, powerY = 0;

            double vMax = 0;
            p2pTaskFlag = true;
            runtime.reset();
            do{
                try {
                    vMax =  runtime.milliseconds() / ACCEL_TOFULL_TIME;
                    vMax = vMax > p2pVel ? Math.abs(p2pVel) : vMax;

                    nowPos = odometry.getRobotPosition();

                    angleError = targetPos.angle - nowPos.angle;
                    disError = p2pDis(targetPos, nowPos);

/*
                    if (Math.abs(disError) < 0.1 && stopAtEnd){
                        //at end, hold the angle more tight

                        rPID.setPID(turnKp,turnKi,turnKd);
                    }
                    else{
                        rPID.setPID(p2pTurnKp,p2pTurnKi,p2pTurnKd);

                    }
*/

                    if (Math.abs(angleError) > angleTolerance) {
                        powerR = rPID.calculate(angleError);
                        powerR = Math.abs(powerR) > Math.abs(turnVel) ? Math.signum(powerR) * Math.abs(turnVel) : powerR;
                        powerR = Math.abs(powerR) < powerRMin ? Math.signum(powerR) * powerRMin : powerR;
                    }
                    else{
                        powerR = 0;
                    }



                    //translate to robot coordinate by deltaX, deltaY

                    deltaX = Math.cos(Math.toRadians(nowPos.angle)) * (targetPos.x - nowPos.x) + Math.sin(Math.toRadians(nowPos.angle)) * (targetPos.y - nowPos.y);
                    deltaY = -Math.sin(Math.toRadians(nowPos.angle)) * (targetPos.x - nowPos.x) + Math.cos(Math.toRadians(nowPos.angle)) * (targetPos.y - nowPos.y);

                    if (Math.abs(deltaX) > Math.abs(disTolerance))
                    {
                        powerX =  fPID.calculate(deltaX);

                        powerX = Math.abs(powerX) > Math.abs(vMax) ? Math.signum(powerX) * Math.abs(vMax) : powerX;
                        powerX = Math.abs(powerX) < Math.abs(endvel) ? Math.signum(powerX) * Math.abs(endvel) : powerX;

                        powerX = Math.abs(powerX) < powerXMin ? Math.signum(powerX) * powerXMin : powerX;
                        // if implement velocity control,
                        powerX = powerX + xVelKp * (powerX - odometry.getRobotLocalVel().velX / MAX_VEL);
                    }
                    else
                    {
                        powerX = 0;
                    }
                    if (Math.abs(deltaY) > Math.abs(disTolerance))
                    {
                        powerY = sPID.calculate(deltaY);
                        powerY = Math.abs(powerY) > Math.abs(vMax) ? Math.signum(powerY) * Math.abs(vMax) : powerY;
                        powerY = Math.abs(powerY) < Math.abs(endvel) ? Math.signum(powerY) * Math.abs(endvel) : powerY;
                        powerY = Math.abs(powerY) < powerYMin ? Math.signum(powerY) * powerYMin : powerY;

                        //if implement vel control
                        powerY = powerY + yVelKp * (powerY - odometry.getRobotLocalVel().velY / MAX_VEL);

                    }
                    else
                    {
                        powerY = 0;
                    }



                    if (this.stopAtEnd)
                    {
                        if (Math.abs(disError) < disTolerance && Math.abs(angleError) < angleTolerance)
                        {
                            p2pTaskFlag = false;
                        }

                    }
                    else
                    {
                        // not care about angle
                        if (Math.abs(disError) < disTolerance )
                        {
                            p2pTaskFlag = false;
                        }
                    }

                    chassis.driveRobotFSR(powerX, powerY, powerR);

/*
                    telemetry.addData("disError", "%2.2f", disError);
                    telemetry.addData("angleError", "%2.2f", angleError);
                    telemetry.addData("powerP", "%2.2f", powerP);
                    telemetry.addData("powerF","%2.2f", powerX);
                    telemetry.addData("powerS", "%2.2f", powerY);
                    telemetry.addData("powerR", "%2.2f", powerR);
                    telemetry.addData("Time", "%.2f", runtime.milliseconds());
                    telemetry.addData("Timeout", "%d", timeOut);
                    if (p2pTaskFlag){
                        telemetry.addLine("true");
                    }
                    else{
                        telemetry.addLine("false");
                    }
                    telemetry.update();
*/
                    Thread.sleep(period);
                }

                // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
                // or by the interrupted exception thrown from the sleep function.
                //catch (InterruptedException e) {goStraightEndFlag = true;}
                // an error occurred in the run loop.
                catch (Exception e) {
                    telemetry.addLine("error happened");
                    telemetry.update();
                }


            } while(!mainTask.isStopRequested() && !interrupted() && p2pTaskFlag && runtime.milliseconds() < timeOut);

            if (this.stopAtEnd){
                chassis.driveRobot(0,0,0,0);
            }
            p2pTaskFlag = false;

        }

    }

    /**
     *
     * @param p1 -- start position
     * @param p2 -- end position
     * @return distance, always > 0
     */
    public static double p2pDis(RobotPosition p1, RobotPosition p2)
    {
        return Math.sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y));
    }

    /**
     *
     * @param angle input angle, in degree
     * @return angle will be in -180 ~ 180 range
     */
    public double angleWrap180(double angle)
    {
        angle = fmod(angle + 180,360);
        if (angle < 0)
            angle += 360;
        return angle - 180;
    }

    public static double fmod(double a, double b) {
        int result = (int) Math.floor(a / b);
        return a - result * b;
    }

    /**
     *
     * @param start -- start position
     * @param end -- end position
     * @return angle radian -- start --> pointing to end, range -pi ~ PI,
     */
    public double p2pAngle(RobotPosition start, RobotPosition end)
    {
        return Math.atan2(end.y - start.y, end.x - start.x);
    }

    enum TURN_WHEEL {
        WHEELS_TURN,    // two wheel turn
        LEFT_WHEEL,     // left wheel turn
        RIGHT_WHEEL     // right wheel turn
    }

    private static class HoldPositionThread extends Thread {
        //point to point drive, x,y and turn at same time
        private static volatile RobotPosition targetPos = new RobotPosition(0,0,0);
        private static volatile double disTolerance;
        private static volatile double angleTolerance;

        private double p2pVel;
        private double turnVel;
        private static PosPIDController rPID = null;
        private static PosPIDController fPID = null;
        private static PosPIDController sPID = null;

        private int period = 10;  // 10ms


        HoldPositionThread(RobotPosition targetPos, double p2pVel, double turnVel, double disTolerance, double angleTolerance)
        {
            this.disTolerance = disTolerance;
            this.angleTolerance = angleTolerance;

            this.p2pVel = p2pVel;
            this.turnVel = turnVel;
            this.targetPos.x = targetPos.x;
            this.targetPos.y = targetPos.y;
            this.targetPos.angle = targetPos.angle;
            powerXMin = 0.15;
            powerYMin = 0.2;
            powerRMin = 0.2;



        }

        // called when tread.start is called. thread stays in loop to do what it does until exit is
        // signaled by main code calling thread.interrupt.
        @Override
        public void run() {

            rPID = new PosPIDController(holdPosTurnKp, holdPosTurnKi,holdPosTurnKd);
            rPID.setOutputRange(-0.5,0.5);
            rPID.setIntegralRange(20);
            rPID.setMaxSumIntegral(holdPosTurnMaxKi);

            fPID = new PosPIDController(holdPosXKp, holdPosXKi,holdPosXKd);
            fPID.setOutputRange(-1.0,1.0);
            fPID.setIntegralRange(0.2);
            fPID.setMaxSumIntegral(holdPosXMaxKi);

            sPID = new PosPIDController(holdPosYKp, holdPosYKi,holdPosYKd);
            sPID.setOutputRange(-1.0,1.0);
            sPID.setIntegralRange(0.2);
            sPID.setMaxSumIntegral(holdPosYMaxKi);


            RobotPosition nowPos = new RobotPosition(0,0,0);

            double theta = 0;


            double deltaX = 0, deltaY = 0;

            RobotVel robotVel = odometry.getRobotLocalVel(); // is in m/s unit

            nowPos = odometry.getRobotPosition();

            double powerX = 0, powerY = 0;

            double vMax = 0;
            runtime.reset();
            do{
                try {
                    //vMax =  runtime.milliseconds() / ACCEL_TOFULL_TIME;
                    //vMax = vMax > p2pVel ? Math.abs(p2pVel) : vMax;
                    vMax = 1.0;
                    nowPos = odometry.getRobotPosition();

                    angleError = this.targetPos.angle - nowPos.angle;
                    disError = p2pDis(this.targetPos, nowPos);

                    rPID.setPID(holdPosTurnKp,holdPosTurnKi,holdPosTurnKd);

                    if (Math.abs(angleError) > angleTolerance) {
                        powerR = rPID.calculate(angleError);
                        powerR = Math.abs(powerR) > Math.abs(turnVel) ? Math.signum(powerR) * Math.abs(turnVel): powerR;
                        powerR = Math.abs(powerR) < powerRMin ? Math.signum(powerR) * powerRMin : powerR;
                    }
                    else{
                        powerR = 0;
                    }



                    //translate to robot coordinate by deltaX, deltaY

                    deltaX = Math.cos(Math.toRadians(nowPos.angle)) * (this.targetPos.x - nowPos.x) + Math.sin(Math.toRadians(nowPos.angle)) * (this.targetPos.y - nowPos.y);
                    deltaY = -Math.sin(Math.toRadians(nowPos.angle)) * (this.targetPos.x - nowPos.x) + Math.cos(Math.toRadians(nowPos.angle)) * (this.targetPos.y - nowPos.y);

                    if (Math.abs(deltaX) > Math.abs(disTolerance))
                    {
                        powerX =  fPID.calculate(deltaX);

                        powerX = Math.abs(powerX) > Math.abs(vMax) ? Math.signum(powerX) * Math.abs(vMax) : powerX;
                        powerX = Math.abs(powerX) < powerXMin ? Math.signum(powerX) * powerXMin : powerX;
                        // if implement velocity control,
                        powerX = powerX + xVelKp * (powerX - odometry.getRobotLocalVel().velX / MAX_VEL);
                    }
                    else
                    {
                        powerX = 0;
                    }
                    if (Math.abs(deltaY) > Math.abs(disTolerance))
                    {
                        powerY = sPID.calculate(deltaY);
                        powerY = Math.abs(powerY) > Math.abs(vMax) ? Math.signum(powerY) * Math.abs(vMax) : powerY;

                        powerY = Math.abs(powerY) < powerYMin ? Math.signum(powerY) * powerYMin : powerY;
                        //if implement vel control
                        powerY = powerY + yVelKp * (powerY - odometry.getRobotLocalVel().velY / MAX_VEL);


                    }
                    else
                    {
                        powerY = 0;
                    }

                    chassis.driveRobotFSR(powerX, powerY, powerR);

/*
                    telemetry.addData("dis", "%2.2f", disError);
                    telemetry.addData("X", "%2.2f", nowPos.x);
                    telemetry.addData("TX", "%2.2f", this.targetPos.x);
                    telemetry.addData("powerx","%2.2f", powerX);
                    telemetry.addData("powerY", "%2.2f", powerY);
                    telemetry.addData("powerR", "%2.2f", powerR);
                    telemetry.update();
*/
                    Thread.sleep(period);
                }

                // interrupted means time to shutdown. note we can stop by detecting isInterrupted = true
                // or by the interrupted exception thrown from the sleep function.
                //catch (InterruptedException e) {goStraightEndFlag = true;}
                // an error occurred in the run loop.
                catch (Exception e) {}
            } while(mainTask.opModeIsActive() && !mainTask.isStopRequested() && !interrupted() && holdPositionTaskFlag);

            chassis.driveRobot(0,0,0,0);

            holdPositionTaskFlag = false;
        }

    }

    public void setHoldPositionTaskOver()
    {
        holdPositionTaskFlag = false;
        mainTask.sleep(50);
        if (holdPositionThread != null){
            holdPositionThread.interrupt();
            holdPositionThread = null;
        }
        powerRMin = 0.05;
        powerYMin = 0.05;
        powerXMin = 0.05;
    }

    public void setHoldPositionTaskStart( RobotPosition holdPosition)
    {
        holdPositionTaskFlag = true;
        holdPositionThread = new HoldPositionThread(holdPosition, 0.2,0.2,0.01,1);
        holdPositionThread.start();
    }

    public void setMinPower(double xPwr, double yPwr, double turnPwr){
        powerXMin = xPwr;
        powerYMin = yPwr;
        powerRMin = turnPwr;
    }

    public void resetP2PPID(){
        p2pXKp = 1.5;
        p2pXKd = 3;
        p2pXKi = 0.08;

        p2pYKp = 3.2;
        p2pYKd = 8.0;
        p2pYKi = 0.1;

        p2pTurnKp = 0.01;
        p2pTurnKi = 0.001;
        p2pTurnKd = 0.05;

    }

    public void resetMinPower(){
        powerXMin = 0.1;
        powerYMin = 0.1;
        powerRMin = 0.1;
    }

}
