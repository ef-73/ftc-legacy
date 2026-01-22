package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.android.dex.Code;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class HSliderSystem {
    private static volatile DcMotorEx sliderMtr1;
    private static volatile DcMotorEx sliderMtr2;


    private static volatile Servo clawPan;
    private static volatile Servo clawTilt1Left;
    private static volatile Servo clawTilt1Right;

    private static volatile Servo clawTilt2;
    private static volatile Servo claw;
    private static volatile Servo block;


    public static volatile TouchSensor hTouch;

    public volatile AnalogInput leftCone = null;
    public volatile AnalogInput rightCone = null;

    private static volatile LinearOpMode mainTask;
    private static volatile Telemetry telemetry;


    private static final double TILT1_SERVO_RES = 1.0 / 162;
    private static final double TILT2_SERVO_RES = 1.0 / 162;
    private static final double PAN_SERVO_RES = 1.0 / 162;
    private static final double CLAW_SERVO_RES = 1.0 / 162;


    //if you switch servo, may need modify these values
    private static final double TILT1_H_ANGLE = -10.8;
    private static final double TILT2_H_ANGLE = -44.55;
    private static double TILT1_ZERO = 0.5 ;//0.5;
    private static double TILT2_ZERO = 0.5; //0.5;
    private static double PAN_ZERO = 0.472;
    private static double CLAW_ZERO = 0.5;

    private static final double TILT1_MAX = 81; // mechanical limitation
    private static final double TILT1_MIN = -81;
    private static final double TILT2_MAX = 81; // mechanical limitation
    private static final double TILT2_MIN = -81;


    private static final double CLAW_MIN = 0;
    private static final double CLAW_MAX = 81;

    public static final double PAN_MAX = 65;  // right
    public static final double PAN_MIN = -65;  // left


    private static final double SLIDER_PPR = 145.1;   /// 435 RPM motor 384.5, 1150 RPM motor 145.1, 1.5 gear ratio
    private static final double PULL_WHEEL_DIA = 0.03565;  //unit : m
    private static final double PULLY_WIRE_DIA = 0.00075;  //unit: m
    private static final int SLIDER_STAGE = 4;
    private static final double SLIDER_STAGE_LEN = 0.244; // unit: m
    public static  final int SLIDER_MIN = 10;

    public static final int SLIDER_MAX =  1800;   // 1840 need test due to the wire length  calculate (int)(SLIDER_STAGE * SLIDER_STAGE_LEN / (Math.PI * (PULL_WHEEL_DIA + PULLY_WIRE_DIA * 3)) * SLIDER_PPR);  // 2400  2300
    //unit: meter
    public static final double SLIDER_LEN_RES = (Math.PI * PULL_WHEEL_DIA) / SLIDER_PPR;  // 2400  2300

    public static final double TILT1_STEP = 1.0;       // degree
    public static final double TILT2_STEP = 0.5;       // degree
    public static final double PAN_STEP = 1.0;        // degree



    public static final double CLAW_CLOSE_ANGLE = CLAW_MAX;

    public static final double CLAW_OPEN_ANGLE = 20;  //10

    //initialize angle
    public static final double SLIDE_BLOCK_SERVO_RELEASE = 0.65;
    public static final double SLIDE_BLOCK_SERVO_BLOCK = 0.5;
    public static final double CLAW_INIT_ANGLE = CLAW_CLOSE_ANGLE - 10;
    private static final double TILT1_INIT_ANGLE = 81;
    private static final double TILT2_INIT_ANGLE = 0;
    private static final double PAN_INIT_ANGLE = 0;

    private static volatile double tilt1Angle = TILT1_INIT_ANGLE;
    private static volatile double tilt2Angle = TILT2_INIT_ANGLE;
    private static volatile double clawAngle = CLAW_INIT_ANGLE;
    private static volatile double panAngle = PAN_INIT_ANGLE;

    private static volatile double sliderLen = 0;

    private final double SLIDER_STEP = 0.01;  // 1cm


    private static volatile int clawDelayCnt = 0;

    private static volatile ResetCascadeThread resetCascade = null;

    private static volatile double sliderBtnCnt = 0;
    private static volatile double sliderPwr = 0;
    private static volatile double sliderMinPwr = 0.3;
    private static volatile int sliderPos = 0;



    public HSliderSystem(LinearOpMode mainTask, HardwareMap hwMap, Telemetry telemetry, boolean auto)
    {
        this.mainTask = mainTask;
        this.telemetry = telemetry;
        sliderMtr1 = hwMap.get(DcMotorEx.class, "hSlide1");
        sliderMtr1.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr1.setPositionPIDFCoefficients(6.0);

        sliderMtr2 = hwMap.get(DcMotorEx.class, "hSlide2");
        sliderMtr2.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr2.setPositionPIDFCoefficients(6.0);


        clawTilt1Left = hwMap.get(Servo.class, "hcTilt1Left");
        clawTilt1Right = hwMap.get(Servo.class, "hcTilt1Right");
        clawTilt1Left.setDirection(Servo.Direction.REVERSE);
        clawTilt1Right.setDirection(Servo.Direction.FORWARD);

        clawTilt2 = hwMap.get(Servo.class, "hcTilt2");
        clawTilt2.setDirection(Servo.Direction.REVERSE);

        claw = hwMap.get(Servo.class, "hcClaw");
        claw.setDirection(Servo.Direction.FORWARD);

        clawPan = hwMap.get(Servo.class, "hcPan");
        clawPan.setDirection(Servo.Direction.FORWARD);

        block = hwMap.get(Servo.class, "hcBlock");
        block.setDirection(Servo.Direction.FORWARD);
        block.setPosition(SLIDE_BLOCK_SERVO_RELEASE);



        hTouch = hwMap.get(TouchSensor.class, "hTouch");

        leftCone = hwMap.get(AnalogInput.class, "leftCone");
        rightCone = hwMap.get(AnalogInput.class, "rightCone");


        readHZeroValue();
        sliderLen = getSliderLen();


        //reset slider if in auto mode

        if (auto) {

            sliderMtr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr1.setPower(-0.5);
            sliderMtr2.setPower(-0.5);
            while (!mainTask.isStopRequested() && !hTouch.isPressed()) {
                mainTask.sleep(10);

            }

            sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mainTask.sleep(200);
            //sliderHoldPos(0);
        }
        else{
            sliderLen = getSliderLen();
        //    sliderLenCtrl(sliderLen, sliderMinPwr);
        }

        if (auto) {
            tilt1Angle = 78;
            setTilt1Angle(tilt1Angle);
            tilt2Angle = 46;
            setTilt2Angle(tilt2Angle);

            panAngle = 0;
            setPanAngle(panAngle);
            clawAngle = CLAW_CLOSE_ANGLE;
            //setClawAngle(clawAngle);

        }
        else{
            // for tele-operation, no move for servo at initialization
            tilt1Angle = 0;
            tilt2Angle = 0;
      //      setTiltAngle(tiltAngle);

            clawAngle = CLAW_CLOSE_ANGLE;
      //      setClawAngle(clawAngle);

            panAngle = 0;
     //       setPanAngle(panAngle);
        }



    }
    private void setSliderMotorKp( double kp){
        sliderMtr1.setPositionPIDFCoefficients(kp);
        sliderMtr2.setPositionPIDFCoefficients(kp);
    }

    private void sliderHoldPos(double pwr){
        sliderPos = sliderMtr1.getCurrentPosition();
        sliderMtr1.setTargetPosition(sliderPos);
        sliderMtr1.setPower(pwr);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderPos = sliderMtr2.getCurrentPosition();
        sliderMtr2.setTargetPosition(sliderPos);
        sliderMtr2.setPower(pwr);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void joystickCtrl(Gamepad gamepad1, Gamepad gamepad2)
    {

        if (gamepad2.left_trigger > 0.5)
        {
            if (sliderMtr1.getCurrentPosition() >= 30 ) {
                sliderBtnCnt += 0.05;
                sliderBtnCnt = sliderBtnCnt > (1- sliderMinPwr) ?   (1- sliderMinPwr) : sliderBtnCnt;
                sliderPwr = -(sliderMinPwr + sliderBtnCnt);
                if (getSliderLen() < 0.1)  sliderPwr = -sliderMinPwr;
                if (sliderMtr1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (sliderMtr2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER){
                    sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                sliderMtr1.setPower(sliderPwr);
                sliderMtr2.setPower(sliderPwr);

            }
            else
            {
                sliderLenCtrl(0.01, 0.1);
                sliderBtnCnt = 0;
            }
        }
        else if(gamepad2.left_bumper){
            if (sliderMtr1.getCurrentPosition() <= SLIDER_MAX) {
                sliderBtnCnt += 0.05;
                sliderBtnCnt = sliderBtnCnt > (1 - sliderMinPwr) ? (1 - sliderMinPwr) : sliderBtnCnt;
                sliderPwr = (sliderMinPwr + sliderBtnCnt);
                if (sliderMtr1.getCurrentPosition() > SLIDER_MAX - 200) sliderPwr = sliderMinPwr;

                if (sliderMtr1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                if (sliderMtr2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                    sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                sliderMtr1.setPower(sliderPwr);
                sliderMtr2.setPower(sliderPwr);
            }
            else{
                sliderHoldPos(sliderMinPwr);
                sliderBtnCnt = 0;
            }

        }
        else{
            if (sliderBtnCnt != 0){
                sliderHoldPos(sliderMinPwr);
                sliderBtnCnt = 0;
            }

        }


        //left joystick control vSlider tilt and pan
        if (Math.abs(gamepad2.left_stick_y) > 0.3){
            tilt1Angle =  tilt1Angle - gamepad2.left_stick_y * TILT1_STEP;
            tilt1Angle = tilt1Angle > TILT1_MAX ? TILT1_MAX : tilt1Angle;
            tilt1Angle = tilt1Angle < TILT1_MIN ? TILT1_MIN : tilt1Angle;
            setTilt1Angle(tilt1Angle);
            if (tilt1Angle < 15){
                tilt2Angle = TILT1_H_ANGLE - tilt1Angle + TILT2_H_ANGLE;
                setTilt2Angle(tilt2Angle);
            }
        }

        if (Math.abs(gamepad2.left_stick_x) > 0.3)
        {

            panAngle =  panAngle - gamepad2.left_stick_x * PAN_STEP;
            panAngle = panAngle > PAN_MAX? PAN_MAX : panAngle;
            panAngle = panAngle < PAN_MIN ? PAN_MIN : panAngle;
            setPanAngle(panAngle);


        }

        if (gamepad1.left_bumper){
            // up for tilt2
            tilt2Angle =  tilt2Angle + TILT2_STEP;
            tilt2Angle = tilt2Angle > TILT2_MAX ? TILT2_MAX : tilt2Angle;
            tilt2Angle = tilt2Angle < TILT2_MIN ? TILT2_MIN : tilt2Angle;
            setTilt2Angle(tilt2Angle);

        }
        if (gamepad1.left_trigger > 0.3){
            tilt2Angle =  tilt2Angle - TILT2_STEP;
            tilt2Angle = tilt2Angle > TILT2_MAX ? TILT2_MAX : tilt2Angle;
            tilt2Angle = tilt2Angle < TILT2_MIN ? TILT2_MIN : tilt2Angle;
            setTilt2Angle(tilt2Angle);
        }







    }

    private void sliderCtrl(int extendPos) {
        sliderCtrl(extendPos, 1.0);

    }

    private void sliderCtrl(int extendPos, double pwr) {
        sliderPos = extendPos;
        sliderPos = sliderPos > SLIDER_MAX ?  SLIDER_MAX: sliderPos;
        sliderPos = sliderPos < SLIDER_MIN ? SLIDER_MIN: sliderPos;

        sliderMtr1.setTargetPosition(sliderPos);
        sliderMtr1.setPower(pwr);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMtr2.setTargetPosition(sliderPos);
        sliderMtr2.setPower(pwr);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }



    private void setClawPosition(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;
        claw.setPosition(pos);

    }



    private void setPanPos(double pos)
    {
        pos = pos < 0 ? 0 : pos;
        pos = pos > 1.0 ? 1.0: pos;

        clawPan.setPosition(pos);

    }

    public boolean isATEnd()
    {
        return hTouch.isPressed();
    }

    public void sliderLenCtrl(double len, double pwr){

        int pos = (int)(len / SLIDER_LEN_RES);
        pos = pos > SLIDER_MAX ?  SLIDER_MAX: pos;
        pos = pos < SLIDER_MIN ? SLIDER_MIN: pos;
        sliderCtrl(pos, pwr);

    }

    public double getSliderLen(){
        double len = sliderMtr1.getCurrentPosition() * SLIDER_LEN_RES;
        return len;
    }


    public void setTilt1Angle(double angle){
        angle = angle > TILT1_MAX ? TILT1_MAX : angle;
        angle = angle < TILT1_MIN ?  TILT1_MIN: angle;
        tilt1Angle = angle;
        double pos  = tilt1Angle * TILT1_SERVO_RES + TILT1_ZERO;
        clawTilt1Left.setPosition(pos);
        clawTilt1Right.setPosition(pos);

    }

    public void setTilt2Angle(double angle){
        angle = angle > TILT2_MAX ? TILT2_MAX : angle;
        angle = angle < TILT2_MIN ?  TILT2_MIN: angle;
        tilt2Angle = angle;
        double pos  = tilt2Angle * TILT2_SERVO_RES + TILT2_ZERO;
        clawTilt2.setPosition(pos);

    }

    public void setPanAngle(double angle){
        angle = angle > PAN_MAX ? PAN_MAX : angle;
        angle = angle < PAN_MIN ? PAN_MIN : angle;
        panAngle = angle;
        double pos  = angle * PAN_SERVO_RES + PAN_ZERO;
        setPanPos(pos);

    }

    public void setClawAngle(double angle){
        angle = angle > CLAW_MAX ? CLAW_MAX : angle;
        angle = angle < CLAW_MIN ? CLAW_MIN : angle;
        double pos = angle * CLAW_SERVO_RES + CLAW_ZERO;
        setClawPosition(pos);
    }

    public double getTilt1Angle(){
        double angle = (clawTilt1Left.getPosition() - TILT1_ZERO) / TILT1_SERVO_RES;
        return angle;
    }

    public double getTilt2Angle(){
        double angle = (clawTilt2.getPosition() - TILT2_ZERO) / TILT2_SERVO_RES;
        return angle;
    }


    public double getPanAngle(){
        double angle = (clawPan.getPosition() - PAN_ZERO) / PAN_SERVO_RES;
        return angle;
    }

    public double getClawAngle(){
        double angle = (claw.getPosition() - CLAW_ZERO) / CLAW_SERVO_RES;
        return angle;
    }


    public void sliderVelCtrl(double pwr)
    {
        if (sliderMtr1.getCurrentPosition() < SLIDER_MAX && sliderMtr1.getCurrentPosition() > SLIDER_MIN){
            sliderMtr1.setPower(pwr);
            sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (sliderMtr2.getCurrentPosition() < SLIDER_MAX && sliderMtr2.getCurrentPosition() > SLIDER_MIN){
            sliderMtr2.setPower(pwr);
            sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }


    public void enableTouchCheck()
    {
        resetCascade = new ResetCascadeThread();
        resetCascade.start();
    }

    private class ResetCascadeThread extends Thread
    {
        public  volatile boolean runningFlag = false;
        public  volatile boolean resetFlag = false;

        public ResetCascadeThread()
        {
            runningFlag = true;
        }

        @Override
        public void run()
        {
            while(!mainTask.opModeIsActive() && runningFlag)
            {
                if(hTouch.isPressed() && !resetFlag)
                {
                    sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderLen = 0.01;
                    sliderLenCtrl(sliderLen, 0.3);
                    resetFlag = true;
                    telemetry.addLine("HSlider Reset slider-----------------");
                    telemetry.update();
                }

                if (Math.abs(sliderMtr1.getCurrentPosition()) > 100){
                    resetFlag = false;
                }

                // check the position is reached or not, 2 is too small?
                if (Math.abs(sliderMtr1.getCurrentPosition() - sliderPos) < 5){
                    sliderCtrl(sliderPos, 0.1);
                }
                mainTask.sleep(100);
            }
        }
    }





    public void slowMoveSliderMtr(double pwr)
    {
        sliderMtr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        sliderMtr1.setPower(pwr);
        sliderMtr2.setPower(pwr);
    }

    public void stopSliderMtr(){
        sliderMtr1.setPower(0);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr2.setPower(0);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // mainTask.sleep(200);
    }

    public int getSliderMtrEncoder(){
        return sliderMtr1.getCurrentPosition();
    }


    public void setSlideBlock(){
        block.setPosition(SLIDE_BLOCK_SERVO_BLOCK);
    }

    public void setSlideBlockServoRelease(){
        block.setPosition(SLIDE_BLOCK_SERVO_RELEASE);
    }

    public boolean getBlockState(){
        if (block.getPosition() < SLIDE_BLOCK_SERVO_BLOCK + 0.01){
            return true;
        }
        else{
            return false;
        }
    }

    public volatile  double leftIniValue = 0.5;
    public volatile  double rightIniValue = 0.5;
    private boolean detetcConeFlag = false;
    public boolean foundCone = false;

    public void initConeDetectValue(){
        /*
        double leftSum = 0;
        double rightSum = 0;
        int cnt = 4;
        for (int i = 0; i < cnt; i++){
            leftSum += leftCone.getVoltage();
            rightSum += rightCone.getVoltage();
        }
        leftIniValue = leftSum / cnt;
        rightIniValue = rightSum / cnt;*/
        leftIniValue = 0;
        rightIniValue = 0;
    }

    public boolean detectConeRunning(){return detetcConeFlag;}

    public void detectCone(){
        detetcConeFlag = true;
        new Thread(new Runnable() {
            double detecTH = 2.0;
            double deltaPanAngle = 10; //10
            int detectCnt = 0;
            int middleCnt = 0;
            int noTouchCnt = 0;


            @Override
            public void run() {
                detetcConeFlag = true;
                foundCone = false;
                while (!mainTask.isStopRequested() && !foundCone) {  //&& mainTask.opModeIsActive()
                    if (Math.abs(leftCone.getVoltage() - leftIniValue) > detecTH && Math.abs(rightCone.getVoltage() - rightIniValue) < detecTH){
                        noTouchCnt = 0;
                        middleCnt = 0;
                        detectCnt++;

                        if (detectCnt > 1){
                            setPanAngle(getPanAngle() - deltaPanAngle);
                            mainTask.sleep(100);
                            foundCone = true;
                            telemetry.addLine("left");

                        }

                    }
                    else if (Math.abs(leftCone.getVoltage() - leftIniValue) < detecTH && Math.abs(rightCone.getVoltage() - rightIniValue) > detecTH){
                        noTouchCnt = 0;
                        middleCnt = 0;
                        detectCnt --;
                        if (detectCnt < -1){
                            setPanAngle(getPanAngle() + deltaPanAngle);
                            mainTask.sleep(100);
                            foundCone = true;
                            telemetry.addLine("right");
                        }

                    }
                    else if (Math.abs(leftCone.getVoltage() - leftIniValue) > detecTH && Math.abs(rightCone.getVoltage() - rightIniValue) > detecTH){
                        noTouchCnt = 0;
                        middleCnt ++;

                        if (middleCnt > 1){
                            foundCone = true;
                        }
                        telemetry.addLine("middle");

                    }
                    else{
                        // no touch
                        middleCnt = 0;
                        detectCnt = 0;
                        noTouchCnt ++;
                        if (noTouchCnt > 1){
                            setSliderMotorKp(12.0);
                            sliderLenCtrl(getSliderLen() + 0.01, 1.0);
                            mainTask.sleep(200);
                            setSliderMotorKp(6.0);
                        }
                    }
                    telemetry.update();

                    mainTask.sleep(10);
                }
                setSliderMotorKp(12.0);
                sliderLenCtrl(getSliderLen() - 0.03, 1.0); // move back a little
                mainTask.sleep(400);
                setSliderMotorKp(6.0);
                detetcConeFlag = false;

            }
        }).start();
    }

    public void stopDetectCone(){
        detetcConeFlag = false;
        foundCone = true;
        mainTask.sleep(30);
    }

    private boolean readHZeroValue()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        File file = new File(dir,"hSlideServo.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                     PAN_ZERO= Double.parseDouble(line);
                }else if(lineCnt == 1){
                    TILT1_ZERO = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    TILT2_ZERO = Double.parseDouble(line);
                }else if(lineCnt == 3){
                    CLAW_ZERO = Double.parseDouble(line);
                }

                lineCnt ++;
            }
            telemetry.update();
            br.close();
            return true;
        }
        catch (IOException e) {
            //You'll need to add proper error handling here
            return false;
        }
    }



}
