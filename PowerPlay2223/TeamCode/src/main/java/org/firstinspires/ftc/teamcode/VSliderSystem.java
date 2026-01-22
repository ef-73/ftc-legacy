package org.firstinspires.ftc.teamcode;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

public class VSliderSystem {
    private static volatile DcMotorEx sliderMtr1;
    private static volatile DcMotorEx sliderMtr2;

    private static volatile Servo clawTiltRight;
    private static volatile Servo clawTiltLeft;
    private static volatile DrXServo clawPan;
    private static volatile Servo clawRotate;
    private static volatile Servo claw;

    public static volatile TouchSensor vTouch;
    private static volatile LinearOpMode mainTask;
    private static volatile Telemetry telemetry;



    private static final double PAN_GEAR_RATIO = 1.5;
    private static final double CLAW_SERVO_RES = 1.0 / 162;
    private static final double TILT_SERVO_RES = 1.0 / 162;
    private static final double PAN_SERVO_RES =   1.0 / (324 * PAN_GEAR_RATIO);  // not really 360
    private static final double ROTATE_SERVO_RES =  0.06 / 90;  // test, //1.0 / (5 * 360) -- 0.06/90
    private static final double VPAN_SPEED = 60.0 * PAN_GEAR_RATIO / 180;

    //if you switch servo, may need modify these values
    private static double TILT_ZERO = 0.5;
    private static double PAN_ZERO = 0.492;  //0.5
    private static double CLAW_ZERO = 0.5;  //0.5
    private static double ROTATE_ZERO = 0.5;


    private static final double TILT_MAX = 81;
    private static final double TILT_MIN = -81;
    private static final double TILT_POS_MAX = 1.0;     // 1.0/180
    private static final double TILT_POS_MIN = 0.0;



    private static final double CLAW_MIN = 0;
    private static final double CLAW_MAX = 72; //74; // 70 close angle
    private static final double PAN_MAX = 162 * PAN_GEAR_RATIO;
    private static final double PAN_MIN = -162 * PAN_GEAR_RATIO;


    public static final double CLAW_ROTATE_MAX = 360;  //  5 turn 65;  //65.8  200 , need replace 5 turn servo
    public static final double CLAW_ROTATE_MIN = -360; //  5 turn 10



    private static final double SLIDER_PPR = 145.1 * 2;  //1150 RPM gear ratio 2
    private static final double PULL_WHEEL_DIA = 0.03565;  //unit : m
    private static final double PULLY_WIRE_DIA = 0.00075;  //unit: m
    private static final int SLIDER_STAGE = 2;
    private static final double SLIDER_STAGE_LEN = 0.244; // unit: m
    public static final int SLIDER_LOWLM = 5;

    public static  final int SLIDER_MAX = 1200; //1240
    public static final double SLIDER_LEN_RES = (Math.PI * PULL_WHEEL_DIA) / SLIDER_PPR;  // 2400  2300



    private static volatile double sliderLen = 0.01;
    private static final double SLIDER_STEP = 0.01;



    public static final double ROTATE_INI_ANGLE = 0.0;
    public static final double TILT_INI_ANGLE = 0.0;  //0.88
    public static final double PAN_INI_ANGLE = 0;

    public static final double CLAW_CLOSE_ANGLE = CLAW_MAX;  //65;  //65.8
    public static final double CLAW_OPEN_ANGLE = CLAW_MIN + 5;   //10


    private static final double TILT_STEP = 2.0;      //degree
    private static final double PAN_STEP = 2.0;       //degree


    private static volatile double tiltAngle = TILT_INI_ANGLE;
    private static volatile double panAngle = PAN_INI_ANGLE;
    private static volatile double clawAngle = CLAW_CLOSE_ANGLE - 10;  //degree
    private static volatile  double rotateAngle = ROTATE_INI_ANGLE;
    private static double ROTATE_STEP = 1.0;




    private static volatile double sliderBtnCnt = 0;
    private static volatile double sliderPwr = 0;
    private static volatile double sliderMinPwr = 0.3;
    private static volatile int sliderPos = 0;

    private static ResetCascade resetCascade = null;


    public VSliderSystem(LinearOpMode mainTask, HardwareMap hwMap, Telemetry telemetry, boolean auto)
    {
        this.mainTask = mainTask;
        this.telemetry = telemetry;
        sliderMtr1 = hwMap.get(DcMotorEx.class, "vSlide1");
        sliderMtr1.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr1.setPositionPIDFCoefficients(8.0);
        sliderMtr2 = hwMap.get(DcMotorEx.class, "vSlide2");
        sliderMtr2.setDirection(DcMotorSimple.Direction.REVERSE);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr2.setPositionPIDFCoefficients(8.0);


        clawTiltRight = hwMap.get(Servo.class, "vcTiltRight");
        clawTiltRight.setDirection(Servo.Direction.FORWARD);
        clawTiltLeft = hwMap.get(Servo.class, "vcTiltLeft");
        clawTiltLeft.setDirection(Servo.Direction.REVERSE);

        clawPan = new DrXServo(mainTask,"vcPan1", "vcPan2",PAN_SERVO_RES,VPAN_SPEED, PAN_ZERO);
        clawPan.setDirection(Servo.Direction.FORWARD, Servo.Direction.FORWARD);


        claw = hwMap.get(Servo.class, "vcClaw");
        claw.setDirection(Servo.Direction.FORWARD);

        clawRotate = hwMap.get(Servo.class, "vcRotate");
        clawRotate.setDirection(Servo.Direction.FORWARD);


        vTouch = hwMap.get(TouchSensor.class, "vTouch");

        readVZeroValue();

        if (auto) {
            tiltAngle = -60;
           // setTiltAngle(tiltAngle);
            panAngle = -28.79;
            setPanAngle(panAngle);
            rotateAngle = -20;
           // setRotateAngle(rotateAngle);
            setClawAngle(CLAW_CLOSE_ANGLE - 4);
        }
        else{
            // tele-operation, no servo movement at initialization
            tiltAngle = 0;
       //     setTiltAngle(tiltAngle);
           // clawTilt.setPosition(0.5);
            panAngle = 0;
       //     setPanAngle(panAngle);

            clawAngle = CLAW_CLOSE_ANGLE;
        //    setClawAngle(clawAngle);

            rotateAngle = 0;
       //     setRotateAngle(rotateAngle);
        }

        sliderLen = getSliderLen();


        if (auto) {

            sliderMtr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr1.setPower(-0.2);
            sliderMtr2.setPower(-0.2);
            while (!mainTask.isStopRequested() && !vTouch.isPressed()) {
                mainTask.sleep(10);

            }
            sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mainTask.sleep(200);
            // sliderHoldPos(0);
        }
        else{
            sliderLen = getSliderLen();
       //     sliderLenCtrl(sliderLen, sliderMinPwr);
        }


    }
    private void sliderHoldPos(double pwr){
        sliderMtr1.setTargetPosition(sliderMtr1.getCurrentPosition());
        sliderMtr1.setPower(pwr);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sliderMtr2.setTargetPosition(sliderMtr2.getCurrentPosition());
        sliderMtr2.setPower(pwr);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void joystickCtrl(Gamepad gamepad1, Gamepad gamepad2)
    {
        if (!(gamepad2.right_bumper && gamepad2.right_trigger > 0.5)) {
            if (gamepad2.right_trigger > 0.5) {
                if (sliderMtr1.getCurrentPosition() >= 30) {
                    sliderBtnCnt += 0.05;
                    sliderBtnCnt = sliderBtnCnt > (1 - sliderMinPwr) ? (1 - sliderMinPwr) : sliderBtnCnt;
                    sliderPwr = -(sliderMinPwr + sliderBtnCnt);
                    if (getSliderLen() < 0.1) sliderPwr = -sliderMinPwr;
                    if (sliderMtr1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    if (sliderMtr2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    sliderMtr1.setPower(sliderPwr);
                    sliderMtr2.setPower(sliderPwr);
                } else {
                    sliderLenCtrl(0.002, 0.3);
                    sliderBtnCnt = 0;
                }
            } else if (gamepad2.right_bumper) {
                if (sliderMtr1.getCurrentPosition() <= SLIDER_MAX) {
                    sliderBtnCnt += 0.05;
                    sliderBtnCnt = sliderBtnCnt > (1 - sliderMinPwr) ? (1 - sliderMinPwr) : sliderBtnCnt;
                    sliderPwr = (sliderMinPwr + sliderBtnCnt);
                    if (sliderMtr1.getCurrentPosition() > SLIDER_MAX - 200)
                        sliderPwr = sliderMinPwr;

                    if (sliderMtr1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    if (sliderMtr2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    }
                    sliderMtr1.setPower(sliderPwr);
                    sliderMtr2.setPower(sliderPwr);
                } else {
                    sliderHoldPos(sliderMinPwr);
                    sliderBtnCnt = 0;
                }

            } else {
                if (sliderBtnCnt != 0) {
                    sliderHoldPos(sliderMinPwr);
                    sliderBtnCnt = 0;
                }
            }
        }


        //right joystick control vSlider tilt and pan
        if (Math.abs(gamepad2.right_stick_y) > 0.3){
            tiltAngle =  tiltAngle - gamepad2.right_stick_y * TILT_STEP;
            tiltAngle = tiltAngle > TILT_MAX ? TILT_MAX : tiltAngle;
            tiltAngle = tiltAngle < TILT_MIN ? TILT_MIN : tiltAngle;
            setTiltAngle(tiltAngle);
        }

        if (Math.abs(gamepad2.right_stick_x) > 0.3)
        {
            panAngle =  panAngle + gamepad2.right_stick_x * PAN_STEP;
            panAngle = panAngle > PAN_MAX ? PAN_MAX : panAngle;
            panAngle = panAngle < PAN_MIN ? PAN_MIN : panAngle;
            setPanAngle(panAngle);
        }






        if (gamepad1.right_stick_y > 0.3){
            rotateAngle = rotateAngle + ROTATE_STEP * Math.abs(gamepad1.right_stick_y);
            rotateAngle = rotateAngle > CLAW_ROTATE_MAX ? CLAW_ROTATE_MAX : rotateAngle;
            setRotateAngle(rotateAngle);
        }
        else if(gamepad1.right_stick_y < -0.3){
            rotateAngle = rotateAngle - ROTATE_STEP * Math.abs(gamepad1.right_stick_y);
            rotateAngle = rotateAngle < CLAW_ROTATE_MIN ? CLAW_ROTATE_MIN : rotateAngle;
            setRotateAngle(rotateAngle);
        }



    }

    private void sliderCtrl(int extendPos) {
       sliderCtrl(extendPos, 1.0);
    }

    private void sliderCtrl(int extendPos, double pwr) {
        sliderPos = extendPos;
        sliderPos = sliderPos > SLIDER_MAX ?  SLIDER_MAX: sliderPos;
        sliderPos = sliderPos < SLIDER_LOWLM ? SLIDER_LOWLM: sliderPos;
        sliderMtr1.setTargetPosition(sliderPos);
        sliderMtr1.setPower(pwr);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        sliderMtr2.setTargetPosition(sliderPos);
        sliderMtr2.setPower(pwr);
        sliderMtr2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




    private void setClawPosition(double pos)
    {
        pos = pos < 0.5 ? 0.5 : pos;
        pos = pos > 1.0 ? 1.0: pos;
        claw.setPosition(pos);
    }


    public void sliderLenCtrl(double len, double pwr){
        int pos = (int)(len / SLIDER_LEN_RES);
        pos = pos > SLIDER_MAX ?  SLIDER_MAX: pos;
        pos = pos < SLIDER_LOWLM ? SLIDER_LOWLM: pos;
        sliderLen = pos * SLIDER_LEN_RES;
        sliderCtrl(pos, pwr);;

    }

    public double getSliderLen(){
        double len = sliderMtr1.getCurrentPosition() * SLIDER_LEN_RES;
        return len;
    }


    public void setTiltAngle(double angle){
        angle = angle > TILT_MAX ? TILT_MAX : angle;
        angle = angle < TILT_MIN ? TILT_MIN : angle;
        tiltAngle = angle;
        double pos = tiltAngle * TILT_SERVO_RES + TILT_ZERO;
        setTiltPosition(pos);

    }

    private void setTiltPosition(double pos){
        pos = pos > TILT_POS_MAX ? TILT_POS_MAX : pos;
        pos = pos < TILT_POS_MIN ? TILT_POS_MIN : pos;
        clawTiltRight.setPosition(pos);
        clawTiltLeft.setPosition(pos);
    }




    public void setPanAngle(double angle){
        angle = angle > PAN_MAX ? PAN_MAX : angle;
        angle = angle < PAN_MIN ? PAN_MIN: angle;
        panAngle = angle;
        clawPan.setTargetAngle(panAngle);

    }
    public void setPanAngle(double angle, double time){
        angle = angle > PAN_MAX ? PAN_MAX : angle;
        angle = angle < PAN_MIN ? PAN_MIN: angle;
        panAngle = angle;
        clawPan.setTargetAngle(panAngle, time);

    }

    public void setClawAngle(double angle){
        angle = angle > CLAW_MAX ? CLAW_MAX : angle;
        angle = angle < CLAW_MIN ? CLAW_MIN : angle;
        double pos = angle * CLAW_SERVO_RES + CLAW_ZERO;
        setClawPosition(pos);
    }

    public double getTiltAngle(){
           return (clawTiltRight.getPosition() - TILT_ZERO) / TILT_SERVO_RES;
    }

    public double getPanAngle(){

        return clawPan.getCurrentAngle();
    }

    public double getClawAngle(){
        double angle = (claw.getPosition() - CLAW_ZERO) / CLAW_SERVO_RES;
        return angle;
    }

    public void setRotateAngle(double angle){
        angle = angle > CLAW_ROTATE_MAX ? CLAW_ROTATE_MAX : angle;
        angle = angle < CLAW_ROTATE_MIN ? CLAW_ROTATE_MIN : angle;
        rotateAngle = angle;
        double pos = rotateAngle * ROTATE_SERVO_RES + ROTATE_ZERO;
        setClawRotatePosition(pos);
    }

    private void setClawRotatePosition(double pos){
        pos = pos > 1.0 ? 1.0 : pos;
        pos = pos < 0 ? 0 : pos;
        clawRotate.setPosition(pos);

    }



    public double getRotateAngle(){
        double angle = (clawRotate.getPosition() - ROTATE_ZERO) / ROTATE_SERVO_RES;
        return angle;
    }

    public void sliderVelCtrl(double pwr)
    {
        if (sliderMtr1.getCurrentPosition() < SLIDER_MAX && sliderMtr1.getCurrentPosition() > SLIDER_LOWLM){
            sliderMtr1.setPower(pwr);
            sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        if (sliderMtr2.getCurrentPosition() < SLIDER_MAX && sliderMtr2.getCurrentPosition() > SLIDER_LOWLM){
            sliderMtr2.setPower(pwr);
            sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void enableTouchCheck()
    {
        resetCascade = new VSliderSystem.ResetCascade();
        resetCascade.start();
    }

    private class ResetCascade extends Thread
    {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        public ResetCascade()
        {
            runningFlag = true;
        }

        @Override
        public void run()
        {
            while(!mainTask.opModeIsActive() && runningFlag)
            {
                if(vTouch.isPressed() && !resetFlag)
                {
                    sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    sliderLen = 0.01;
                    sliderLenCtrl(sliderLen, 0.1);
                    resetFlag = true;
                    telemetry.addLine("VSlider Reset slider-----------------");
                    telemetry.update();
                }

                if (Math.abs(sliderMtr1.getCurrentPosition()) > 100){
                    resetFlag = false;
                }
                // check the position is reached or not, 2 is too small?
                if (Math.abs(sliderMtr1.getCurrentPosition() - sliderPos) < 10){
                    sliderCtrl(sliderPos, 0.7);
                }
                mainTask.sleep(100);
            }
        }
    }






    public void setSlideMtrKp(double kp){
        sliderMtr1.setPositionPIDFCoefficients(kp);
        sliderMtr2.setPositionPIDFCoefficients(kp);
    }

    public void stopSlideMtr(){
        sliderMtr1.setPower(0);
        sliderMtr2.setPower(0);

    }

    private boolean readVZeroValue()
    {
        File sdCard = Environment.getExternalStorageDirectory();
        File dir = new File (sdCard.getAbsolutePath() + "/FIRST/ParkPos");
        File file = new File(dir,"vSlideServo.txt");

        StringBuilder text = new StringBuilder();

        try {
            BufferedReader br = new BufferedReader(new FileReader(file));
            String line;

            int lineCnt = 0;
            while ((line = br.readLine()) != null) {
                if (lineCnt == 0){
                    PAN_ZERO= Double.parseDouble(line);
                }else if(lineCnt == 1){
                    TILT_ZERO = Double.parseDouble(line);
                }else if(lineCnt == 2){
                    ROTATE_ZERO = Double.parseDouble(line);
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
