package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SliderAndTraySystem_Servo {
    private ElapsedTime runTime = new ElapsedTime();

    private boolean movingFlag = false;
    private SlideTouchCheck slideTouchCheck = null;


    double MOVE_TRAY_LEN = 0.1;
    private final double RESET_LM_LEN = 0.15;

    private final boolean USE_BACK_DROP = false;

    /*
    double[] LEVEL_LEN = new double[]{0.154, 0.154, 0.23, 0.29, 0.33, 0.408, 0.48};
    double[] LEVEL_ARM_ANGLE = new double[]{-56.062, -70.104, -77.3, -81.451, -90.483, -90.483, -97.715};// second -70.104 - 3.0
    double[] LEVEL_TRAY_ANGLE = new double[]{-90, -73.4, -68.6 + 3.0, -62.4, -52.8, -50, -50};  // second -73.4
    double[] AUTO_LEVEL_LEN = new double[]{0.152, 0.23, 0.152};
    double[] AUTO_LEVEL_ARM_ANGLE = new double[]{-43.0 - 3, -77.4, -66.8};  // third for second level for auto far side drop
    double[] AUTO_LEVEL_TRAY_ANGLE = new double[]{-94 + 1, -65.6, -74.4};
    */

/*
    double[] LEVEL_LEN = new double[]{0.161, 0.164, 0.175, 0.224, 0.306, 0.397,0.49};
    double[] LEVEL_ARM_ANGLE = new double[]{31.86, 58.86, 70.74, 81.0, 81.0, 81.0, 81.0};// second -70.104 - 3.0
    double[] LEVEL_TRAY_ANGLE = new double[]{0.270, 23.67, 33.21, 42.39, 42.39, 42.39, 42.39};  // second -73.4


 */
    /*
    double[] LEVEL_LEN = new double[]{0.17 + 0.02, 0.161+ 0.02, 0.188+ 0.02, 0.241 + 0.02, 0.289 + 0.02, 0.336 + 0.02,0.49 + 0.02};
    double[] LEVEL_ARM_ANGLE = new double[]{31.86, 44.1, 58.58, 66.69, 76.86, 81.0, 81.0};// second -70.104 - 3.0
    double[] LEVEL_TRAY_ANGLE = new double[]{0.27, 8.55, 23.67, 31.13, 40.5, 49.23, 49.23};  // second -73.4
    */
    double[] LEVEL_LEN = new double[]{0.155, 0.183 - 0.02, 0.206- 0.02, 0.244 - 0.00, 0.32 - 0.02, 0.415 - 0.01,0.47 + 0.00};
    double[] LEVEL_ARM_ANGLE = new double[]{42.3 - 4.86, 49.5 - 4.86, 66.15 - 4.86, 68.670 , 81.0 - 4.86 , 81.0, 81.0};// second -70.104 - 3.0
    double[] LEVEL_TRAY_ANGLE = new double[]{-13.68, -6.75, 10.62, 21.96, 23.49, 27.36, 27.36};  // second -73.4


    double[] AUTO_LEVEL_LEN = new double[]{0.170 - 0.015, 0.251 - 0.015, 0.198 - 0.015};
    double[] AUTO_LEVEL_ARM_ANGLE = new double[]{31.86 - 4.86, 58.5 - 4.86, 44.01 - 4.86};  // third for second level for auto far side drop
    double[] AUTO_LEVEL_TRAY_ANGLE = new double[]{-24.86, -3.51, -14.76};


    public double AUTO_LEVEL_1_RELEASE_ARM_ANGLE = -49; //-49

    /*
    // not used now
    double[] LEVEL_LEN = new double[]{0.154, 0.154, 0.23, 0.27, 0.311, 0.408, 0.47};
    double[] LEVEL_ARM_ANGLE = new double[]{-50.062, -70.104, -70.0, -79.164, -90.483, -90.483, -100.425};
    double[] LEVEL_TRAY_ANGLE = new double[]{-77, -60, -60, -50.6, -39, -38, -27.8};
    */


    public final double AUTODROP_PRELOAD_LEN = 0.155;
    public final double AUTODROP_PRELOAD_ARM_ANGLE = 35.108;
    public final double AUTODROP_PRELOAD_TRAY_TILT_ANGLE = -47.6;

    private static volatile ChassisSystem chassisController;
    private static volatile DcMotorEx sliderMtr1;
  //  private static volatile DcMotorEx sliderMtr2;
    private static volatile DcMotorEx intakeMotor;

    public static volatile Servo armLeftTiltServo;
    public static volatile Servo armRightTiltServo;

    private static volatile Servo trayTiltServo;

    private static volatile Servo frontLockServo;
    private static volatile Servo backLockServo;
    private static volatile Servo trayRotateServo;
    private static volatile Servo trayHSlideServo;




    public static volatile TouchSensor resetTouch = null;   // for reset slide
    public static volatile DigitalChannel touchBoardSensor = null;  // for detect touch board or not

    private static volatile LinearOpMode mainTask;
    private static volatile Telemetry telemetry;
    private boolean auto = false;


    private static final double ARM_TILT_SERVO_RES = 1.0 / 162 ; //  switch to 180 servo  1.0 / 360;
    private static final double ARM_TILT_SERVO_SPEED = 60.0  / 200;

    //if you switch servo, may need modify these values
    private static double ARM_TILT_ZERO = 0.5;
    private static final double ARM_TILT_MAX = 81;
    private static final double ARM_TILT_MIN = -81; //-71.28;
    private static final double ARM_TILT_POS_MAX = 1.0;     // 1.0/180
    private static final double ARM_TILT_POS_MIN = 0.058;
    public static final double ARM_TILT_INI_ANGLE = -81; //-76.14; //-79.38 ; //-71.28; //84.6;  //  78.5-- 0.218 + 0.5
    private static final double ARM_TILT_STEP = 2.0;      //degree
    private static final double ARM_TILT_RIGHT_OFFSET = 0.00;
    private static volatile double armTiltAngle = ARM_TILT_INI_ANGLE;


    private static final double TRAY_TILT_SERVO_RES = 1.0 / 162;
    //if you switch servo, may need modify these values   360 degree Servo
    private static double TRAY_TILT_ZERO = 0.5;
    private static final double TRAY_TILT_MAX = 81;
    private static final double TRAY_TILT_MIN = -81;
    private static final double TRAY_TILT_POS_MAX = 1.0;     // 1.0/180

    private static final double TRAY_TILT_POS_MIN = 0.0;

    private static final double TRAY_TILT_STEP = 2.0;
    private static final double TRAY_TILT_INI_ANGLE =  -70; //-75.33; //-51.03 ;//-51.84;
    private static volatile double trayTiltAngle = TRAY_TILT_INI_ANGLE;



    private static final double SLIDER_MIN_POWER = 0.4;
    private static final double gearRatio = 1.0;   //gear ratio = 1:1
    private static final double SLIDER_PPR = 384.5 * gearRatio;  //1150RPM:145.1, 435RPM:  384.5
    private static final double PULLEY_WHEEL_DIA = 0.03565;  //unit : m
    private static final double PULLYEY_WIRE_DIA = 0.00075;  //unit: m
    private static final double PULLEY_WHEEL_PERIMETER = 0.112 ;// perimeter of pully wheel
    private static final int SLIDER_STAGE = 2;
    private static final double SLIDER_STAGE_LEN = 0.244; // unit: m
    public static final int SLIDER_LOWLM = -200;


    public static final double SLIDER_MAX_LEN =  0.50 - 0.01;  // 3 stage, 0.1746 *3 =  0.5292   SLIDER_MAX * SLIDER_LEN_RES;
    public static final double SLIDER_LEN_RES = (PULLEY_WHEEL_PERIMETER) / SLIDER_PPR;
    public static  final int SLIDER_MAX = (int)(SLIDER_MAX_LEN / SLIDER_LEN_RES) ; //1682

    private static volatile double sliderLen = 0.01;
    private static final double SLIDER_STEP = 0.01;

    private final double TRAY_OPEN_POS = 0.0;
    private final double TRAY_HOLD_POS = 0.25;
    private final double TRAY_CLOSE_POS = 0.415;


    private final double ROATE_MAX_ANGLE = 160;
    private final double ROATE_MIN_ANGLE = -160;
    private final double ROTATE_INIT_ANGLE = 0;
    private final double ROTATE_LEFT_ANGLE = 90;
    private final double ROTATE_RIGHT_ANGLE = -90;

    private final double TRAY_ROTATE_RES = 1.0 / 324.0;   // 270 - gobilda servo, 324 -- 360 degree servo
    private final double ROATE_MAX_POS = 1.0;
    private final double ROTATE_MIN_POS = 0.0;

    private final double TRAY_ROTATE_ZERO = 0.495;

    private double trayRotateAngle = ROTATE_INIT_ANGLE;
    public int trayRotatePos = 0; // 1 -- left Pos, 0 -- init pos  ,2 -- right Pos


    private static volatile double sliderBtnCnt = 0;
    private static volatile double sliderPwr = 0;
    private static volatile double sliderMinPwr = 0.15;
    private static volatile int sliderPos = 0;


    private static int intakeMovingFlag = 0;  // 0- stop, -1 intake out, 1-- intake in
    private int dropLevel = -1;

    private int delayButtonACnt = 0;
    private int delayJoystickCnt = 0;

    public DigitalChannel frontRedLed = null;
    public DigitalChannel frontGreenLed = null;

    public DigitalChannel backRedLed = null;
    public DigitalChannel backGreenLed = null;

    private int left_stick_button_Cnt = 0;


    private double TRAY_HSLIDE_INI = 0.5;
    private double TRAY_HSLIDE_MAX = 1.0;
    private double TRAY_HSLIDE_MIN = 0.0;
    private double trayHSlidePos = TRAY_HSLIDE_INI;
    private double trayHSlideStep = 0.05;


    public SliderAndTraySystem_Servo(LinearOpMode mainTask, ChassisSystem chassis, boolean auto)
    {
        this.chassisController = chassis;
        this.mainTask = mainTask;
        this.telemetry = mainTask.telemetry;
        this.auto = auto;


        resetTouch = mainTask.hardwareMap.get(TouchSensor.class, "resetTouch");

        touchBoardSensor = mainTask.hardwareMap.get(DigitalChannel.class, "touchBoardSensor");
        touchBoardSensor.setMode(DigitalChannel.Mode.INPUT);


        //motor
        sliderMtr1 = mainTask.hardwareMap.get(DcMotorEx.class, "slider");
        sliderMtr1.setDirection(DcMotorSimple.Direction.FORWARD);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderMtr1.setPositionPIDFCoefficients(8.0);  //? too big???


        intakeMotor = mainTask.hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Servo
        armLeftTiltServo = mainTask.hardwareMap.get(Servo.class, "leftTilt");
        armLeftTiltServo.setDirection(Servo.Direction.FORWARD);
        armRightTiltServo = mainTask.hardwareMap.get(Servo.class, "rightTilt");
        armRightTiltServo.setDirection(Servo.Direction.REVERSE);


        trayTiltServo = mainTask.hardwareMap.get(Servo.class, "trayTilt");
        trayTiltServo.setDirection(Servo.Direction.REVERSE);




        frontLockServo = mainTask.hardwareMap.get(Servo.class, "frontLock");
        frontLockServo.setDirection(Servo.Direction.FORWARD);

        backLockServo = mainTask.hardwareMap.get(Servo.class, "backLock");
        backLockServo.setDirection(Servo.Direction.FORWARD);

        trayRotateServo = mainTask.hardwareMap.get(Servo.class, "trayRotate");
        trayRotateServo.setDirection(Servo.Direction.FORWARD);

        trayHSlideServo = mainTask.hardwareMap.get(Servo.class, "trayHSlide");
        trayHSlideServo.setDirection(Servo.Direction.FORWARD);



        if (auto) {
            mainTask.sleep(100);
            armTiltAngle = ARM_TILT_INI_ANGLE;
            setArmTiltAngle(armTiltAngle);
            mainTask.sleep(100);
            trayTiltAngle = TRAY_TILT_INI_ANGLE;
            setTrayTargetAngle(trayTiltAngle);
            mainTask.sleep(100);
            trayOpenBothLock();
            mainTask.sleep(100);
            setTrayRotateAngle(ROTATE_INIT_ANGLE);
            dropLevel = -1;
            trayHSlideServo.setPosition(TRAY_HSLIDE_INI);

        }
        else{
            // tele-operation, no servo movement at initialization


            // comment out later

            dropLevel = -1;
        }




        if (auto) {
            mainTask.sleep(200);
            sliderMtr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
           // sliderMtr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            sliderMtr1.setPower(-0.5);
            //mainTask.sleep(500);
            //sliderMtr2.setPower(-0.2);

            while (!mainTask.isStopRequested() && !resetTouch.isPressed()) {
                mainTask.sleep(10);
                mainTask.telemetry.addLine("not touched");
                mainTask.telemetry.update();
            }

            sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
           // sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            mainTask.sleep(200);
            sliderLen = getSliderLen();

        }
        else{
            sliderLen = getSliderLen();
        }
        slideTouchCheck = new SlideTouchCheck();
        if (!auto){
            initLed();
        }



    }
    private void sliderHoldPos(double pwr){
        sliderMtr1.setTargetPosition(sliderMtr1.getCurrentPosition());
        sliderMtr1.setPower(pwr);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void joystickCtrl()
    {
        if (!movingFlag) {
            if (!(mainTask.gamepad2.right_bumper && mainTask.gamepad2.right_trigger > 0.5)) {
                if (mainTask.gamepad2.right_trigger > 0.5) {

                    if (sliderMtr1.getCurrentPosition() >= 10) {
                    /*
                    sliderLen = sliderLen - 0.005;
                    sliderLen = sliderLen < 0.005 ? 0.005 : sliderLen;
                    sliderLenCtrl(sliderLen, 1.0);
                    */


                        sliderBtnCnt += 0.05;
                        sliderBtnCnt = sliderBtnCnt > (1 - sliderMinPwr) ? (1 - sliderMinPwr) : sliderBtnCnt;
                        sliderPwr = -(sliderMinPwr + sliderBtnCnt);

                        if (getSliderLen() < 0.3) sliderPwr = -1.0 * getSliderLen(); // kp

                        sliderPwr = Math.abs(sliderPwr) < SLIDER_MIN_POWER ? -SLIDER_MIN_POWER : sliderPwr;  // kp?

                        if (sliderMtr1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                            sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                        //      if (sliderMtr2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        //          sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        //      }
                        sliderMtr1.setPower(sliderPwr);
                        //      sliderMtr2.setPower(sliderPwr);


                    } else {
                        sliderLenCtrl(0.002, 0.3);
                        sliderBtnCnt = 0;
                    }
                } else if (mainTask.gamepad2.right_bumper) {
                    if (sliderMtr1.getCurrentPosition() <= SLIDER_MAX) {

                    /*
                    sliderLen = sliderLen + 0.005;
                    sliderLen = sliderLen > SLIDER_MAX_LEN ? SLIDER_MAX_LEN : sliderLen;
                    sliderLenCtrl(sliderLen, 1.0);

                    */

                        sliderBtnCnt += 0.05;
                        sliderBtnCnt = sliderBtnCnt > (1 - sliderMinPwr) ? (1 - sliderMinPwr) : sliderBtnCnt;
                        sliderPwr = (sliderMinPwr + sliderBtnCnt);

                        if (getSliderLen() > (SLIDER_MAX_LEN - 0.3))
                            sliderPwr = 1.0 * (SLIDER_MAX_LEN - getSliderLen());

                        sliderPwr = sliderPwr < SLIDER_MIN_POWER ? SLIDER_MIN_POWER : sliderPwr;

                        if (sliderMtr1.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                            sliderMtr1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        }
                        //    if (sliderMtr2.getMode() != DcMotor.RunMode.RUN_USING_ENCODER) {
                        //        sliderMtr2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        //     }
                        sliderMtr1.setPower(sliderPwr);
                        //      sliderMtr2.setPower(sliderPwr);
                    } else {
                        sliderHoldPos(SLIDER_MIN_POWER);
                        sliderBtnCnt = 0;
                    }

                } else {
                    if (sliderBtnCnt != 0) {
                        sliderHoldPos(SLIDER_MIN_POWER);
                        sliderBtnCnt = 0;
                    }
                }
            }


            if (Math.abs(mainTask.gamepad2.left_stick_y) > 0.5) {
                if (getSliderLen() > RESET_LM_LEN){
                    armTiltAngle = armTiltAngle - mainTask.gamepad2.left_stick_y * ARM_TILT_STEP;
                    armTiltAngle = armTiltAngle > ARM_TILT_MAX ? ARM_TILT_MAX : armTiltAngle;
                    armTiltAngle = armTiltAngle < ARM_TILT_MIN ? ARM_TILT_MIN : armTiltAngle;
                    setArmTiltAngle(armTiltAngle);
                }

            }


            if (mainTask.gamepad2.x) {
             //   if (!touchBoardSensor.getState()) {
                    if (getTrayRoateAngle() >= 0) {
                        trayOpenFrontLock();
                    }else{
                        trayOpenBackLock();
                    }
             //   }

            } else if (mainTask.gamepad2.b) {
            //    if (!touchBoardSensor.getState()){
                    if (getTrayRoateAngle() < 0) {
                        trayOpenFrontLock();
                    }else{
                        trayOpenBackLock();
                    }
            //    }
            }else if(mainTask.gamepad2.y){
            //    if (!touchBoardSensor.getState()){
                    trayOpenBothLock();
            //    }
            }

            if (mainTask.gamepad2.a){
                if (getSliderLen() > RESET_LM_LEN) {


                    if (delayButtonACnt <= 0) {
                        delayButtonACnt = 10;
                        trayRotatePos++;

                        int pos = trayRotatePos % 3;
                        trayRotatePos = pos;
                        if (pos == 0) {

                            setTrayRotateAngle(ROTATE_INIT_ANGLE);
                        } else if (pos == 1) {
                            setTrayRotateAngle(ROTATE_LEFT_ANGLE);
                        } else {
                            setTrayRotateAngle(ROTATE_RIGHT_ANGLE);
                        }
                    }
                }


            }
            else{
                if (delayButtonACnt > 0){
                    delayButtonACnt --;
                }

            }






            if (Math.abs(mainTask.gamepad2.right_stick_y) > 0.5) {
                if (getSliderLen() > RESET_LM_LEN){
                    trayTiltAngle = trayTiltAngle - mainTask.gamepad2.right_stick_y * TRAY_TILT_STEP;
                    trayTiltAngle = trayTiltAngle > TRAY_TILT_MAX ? TRAY_TILT_MAX : trayTiltAngle;
                    trayTiltAngle = trayTiltAngle < TRAY_TILT_MIN ? TRAY_TILT_MIN : trayTiltAngle;
                    setTrayTiltAngle(trayTiltAngle);
                }

            }


            if (mainTask.gamepad2.right_stick_x < -0.5 && getSliderLen() > 0.15){
                if (delayJoystickCnt <= 0){
                    delayJoystickCnt = 4;
                    trayRotateAngle = getTrayRoateAngle();
                    trayRotateAngle -= 60;
                    trayRotateAngle = trayRotateAngle < -150 ? -150: trayRotateAngle;
                    setTrayRotateAngle(trayRotateAngle);
                }


            }else if(mainTask.gamepad2.right_stick_x > 0.5 && getSliderLen() > 0.15){
                if (delayJoystickCnt <= 0) {
                    delayJoystickCnt = 4;
                    trayRotateAngle = getTrayRoateAngle();
                    trayRotateAngle += 60;
                    trayRotateAngle = trayRotateAngle > 150 ? 150 : trayRotateAngle;
                    setTrayRotateAngle(trayRotateAngle);
                }
            }
            else{
                if (delayJoystickCnt > 0){
                    delayJoystickCnt --;
                }
            }



            if (mainTask.gamepad1.a) {
                //move forward a little bit first

                mainTask.sleep(200);
                resetArmToPickupPos();

            }

            if (mainTask.gamepad2.left_stick_button){
                if (left_stick_button_Cnt == 0) {
                    left_stick_button_Cnt = 10;
                    if (frontLockServo.getPosition() > 0.2) {
                        trayOpenBothLock();
                    } else {
                        trayCloseBothLock();
                    }
                }
            }
            else{
                if (left_stick_button_Cnt > 0){
                    left_stick_button_Cnt --;
                }
            }




            if (mainTask.gamepad2.dpad_down) {
                toDropLevelPos(0);
            } else if (mainTask.gamepad2.dpad_left) {
                toDropLevelPos(1);
            } else if (mainTask.gamepad2.dpad_up) {
                toDropLevelPos(2);
            } else if (mainTask.gamepad2.dpad_right) {
                toDropLevelPos(3);
            }else if (mainTask.gamepad2.left_bumper) {
                toDropLevelPos(4);
            }else if (mainTask.gamepad2.left_trigger > 0.5) {
                toDropLevelPos(5);
            }else if (mainTask.gamepad2.start) {
                toDropLevelPos(6);
            }


        }

        if (mainTask.gamepad1.left_bumper) {
            if (getSliderLen() < 0.02 && !movingFlag) {
                intakeIn();
            }

        } else if (mainTask.gamepad1.left_trigger > 0.5) {
            if (!movingFlag) {
                intakeOut();
            }

        } else {
            intakeStop();
        }


        /*
        if (Math.abs(mainTask.gamepad2.left_stick_x ) > 0.3){
            if (getSliderLen() > RESET_LM_LEN) {
                double pos = trayHSlidePos - mainTask.gamepad2.left_stick_x * trayHSlideStep;
                setHSlidePos(pos);
            }

        }

         */
        if (mainTask.gamepad1.dpad_left ){
            if (getSliderLen() > RESET_LM_LEN) {
                double pos = trayHSlidePos +   trayHSlideStep;
                setHSlidePos(pos);
            }

        }else  if (mainTask.gamepad1.dpad_right ){
            if (getSliderLen() > RESET_LM_LEN) {
                double pos = trayHSlidePos -  trayHSlideStep;
                setHSlidePos(pos);
            }

        }


    }

    private void setHSlidePos(double pos){
        pos = pos > TRAY_HSLIDE_MAX ? TRAY_HSLIDE_MAX : pos;
        pos = pos < TRAY_HSLIDE_MIN ? TRAY_HSLIDE_MIN : pos;
        trayHSlidePos = pos;
        trayHSlideServo.setPosition(trayHSlidePos);
    }

    public void resetHSlide(){
        trayHSlidePos = TRAY_HSLIDE_INI;
        trayHSlideServo.setPosition(trayHSlidePos);
    }







    public void resetSlider(){
        sliderLenCtrl(-0.01, 1.0);  // make sure touch the touch sensor, then stop motor in reset check thread
    }

    public void intakeIn(){
        trayOpenBothLock();
        intakeMotor.setPower(1.0);
        intakeMovingFlag = 1;
        //setSliderPwr(0);
    }

    public void intakeInWithYellow(){
        frontLockServo.setPosition(TRAY_OPEN_POS);
        backLockServo.setPosition(TRAY_CLOSE_POS);
        intakeMotor.setPower(1.0);
        intakeMovingFlag = 1;
    }

    public void intakeOut(){
        trayCloseBothLock();
        mainTask.sleep(100);
        intakeMotor.setPower(-1.0);
        intakeMovingFlag = -1;

    }

    public void intakeStop(){
        intakeMotor.setPower(0);
        if (intakeMovingFlag == 1) {
            new Thread(new Runnable() {
                @Override
                public void run() {
                    trayCloseBothLock();
                    mainTask.sleep(200);
                    intakeMotor.setPower(-1.0);
                    mainTask.sleep(500);
                    intakeMotor.setPower(0.0);
                    intakeMovingFlag = 0;
                }
            }).start();

        }
        else{
            intakeMotor.setPower(0);
        }
    }

    private void sliderCtrl(int extendPos, double pwr) {
        sliderPos = extendPos;
        sliderPos = sliderPos > SLIDER_MAX ?  SLIDER_MAX: sliderPos;
        sliderPos = sliderPos < SLIDER_LOWLM ? SLIDER_LOWLM: sliderPos;
        sliderMtr1.setTargetPosition(sliderPos);
        sliderMtr1.setPower(pwr);
        sliderMtr1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

       // sliderMtr2.setTargetPosition(sliderPos);
       // sliderMtr2.setPower(pwr);
       // sliderMtr2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void sliderLenCtrl(double len, double pwr){
        int pos = (int)(len / SLIDER_LEN_RES);
        pos = pos > SLIDER_MAX ?  SLIDER_MAX: pos;
        pos = pos < SLIDER_LOWLM ? SLIDER_LOWLM: pos;
        sliderLen = pos * SLIDER_LEN_RES;
        sliderCtrl(pos, pwr);;

    }

    public void setArmTiltAngle(double angle){
        angle = angle > ARM_TILT_MAX ? ARM_TILT_MAX : angle;
        angle = angle < ARM_TILT_MIN ? ARM_TILT_MIN : angle;
        armTiltAngle = angle;
        double pos = armTiltAngle * ARM_TILT_SERVO_RES + ARM_TILT_ZERO;
        armLeftTiltServo.setPosition(pos);
        armRightTiltServo.setPosition(pos + ARM_TILT_RIGHT_OFFSET);

    }


    public int getSliderPos(){
        return sliderMtr1.getCurrentPosition();
    }

    public void setSliderPwr(double pwr){
        sliderMtr1.setPower(pwr);
    }

    public double getSliderLen(){
        double len = sliderMtr1.getCurrentPosition() * SLIDER_LEN_RES;
        return len;
    }

    public void setTrayTargetAngle(double angle){
        angle = angle > TRAY_TILT_MAX ? TRAY_TILT_MAX : angle;
        angle = angle < TRAY_TILT_MIN ? TRAY_TILT_MIN : angle;
        trayTiltAngle = angle;
        double pos = trayTiltAngle * TRAY_TILT_SERVO_RES + TRAY_TILT_ZERO;
        trayTiltServo.setPosition(pos);
    }


    public void setTrayRotateAngle(double angle){
        angle = angle > ROATE_MAX_ANGLE ? ROATE_MAX_ANGLE : angle;
        angle = angle < ROATE_MIN_ANGLE ? ROATE_MIN_ANGLE : angle;
        trayRotateAngle = angle;
        double pos = trayRotateAngle * TRAY_ROTATE_RES + TRAY_ROTATE_ZERO;
        setTrayRotatePosition(pos);


    }

    private void  setTrayRotatePosition(double pos){
        pos = pos > ROATE_MAX_POS ? ROATE_MAX_POS : pos;
        pos = pos < ROTATE_MIN_POS ? ROTATE_MIN_POS : pos;
        trayRotateServo.setPosition(pos);
    }

    public double getTrayRoateAngle(){
        double angle = (trayRotateServo.getPosition() - TRAY_ROTATE_ZERO) / TRAY_ROTATE_RES;
        return angle;
    }

    public void setTrayTiltAngle(double angle){
        angle = angle > TRAY_TILT_MAX ? TRAY_TILT_MAX : angle;
        angle = angle < TRAY_TILT_MIN ? TRAY_TILT_MIN : angle;
        trayTiltAngle = angle;
        double pos = trayTiltAngle * TRAY_TILT_SERVO_RES + TRAY_TILT_ZERO;
        setTrayTiltPosition(pos);

    }

    private void setTrayTiltPosition(double pos){
        pos = pos > TRAY_TILT_POS_MAX ? TRAY_TILT_POS_MAX : pos;
        pos = pos < TRAY_TILT_POS_MIN ? TRAY_TILT_POS_MIN : pos;
        trayTiltServo.setPosition(pos);
    }






    public double getArmTiltAngle(){
        double angle = (armLeftTiltServo.getPosition() - ARM_TILT_ZERO) / ARM_TILT_SERVO_RES;

        return angle;
    }

    public double getTrayTiltAngle(){
        double angle = (trayTiltServo.getPosition() - TRAY_TILT_ZERO) / TRAY_TILT_SERVO_RES;
        return angle;
    }



    public void setSlideMtrKp(double kp){
        sliderMtr1.setPositionPIDFCoefficients(kp);
      //  sliderMtr2.setPositionPIDFCoefficients(kp);
    }

    public void stopSlideMtr(){
        sliderMtr1.setPower(0);
      //  sliderMtr2.setPower(0);

    }

    public void startResetCheck(){
        slideTouchCheck.start();
    }


    private class SlideTouchCheck extends Thread
    {
        public boolean runningFlag = false;
        public boolean resetFlag = false;

        public SlideTouchCheck()
        {
            runningFlag = true;
            resetFlag = false;
        }

        @Override
        public void run()
        {
            while(mainTask.opModeIsActive() && runningFlag)
            {
                if(resetTouch.isPressed() && !resetFlag )
                {
                    sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    //sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    /*
                    // do not hold, otherwise the motor my be overheat
                    sliderLen = 0.01;
                    sliderLenCtrl(sliderLen, 0.1);*/
                    resetFlag = true;
                  //  telemetry.addLine("Slider Reset slider-----------------");

                    sliderBtnCnt = 0;
                }

                if (Math.abs(sliderMtr1.getCurrentPosition()) > 100){
                    resetFlag = false;
                }
                // check the position is reached or not, 2 is too small?
                /*
                if (Math.abs(sliderMtr1.getCurrentPosition() - sliderPos) < 10){
                    sliderCtrl(sliderPos, 0.7);
                }*/
              // telemetry.addLine("testing.....");
              //  telemetry.update();
                mainTask.sleep(100);
            }
        }
    }


    public void resetArmToPickupPos(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                trayOpenBothLock();
                long waitTime = 300;
                movingFlag = true;

                if (Math.abs(trayHSlideServo.getPosition() - 0.5) > 0.55){
                    setHSlidePos(TRAY_HSLIDE_INI);
                    mainTask.sleep(800); // give more time for hslide move
                }
                else if (Math.abs(trayHSlideServo.getPosition() - 0.5) > 0.25){
                      setHSlidePos(TRAY_HSLIDE_INI);
                      mainTask.sleep(500); // give more time for hslide move
                }
                if (Math.abs(getTrayRoateAngle() )> 90 || Math.abs(trayHSlideServo.getPosition() - 0.5) > 0.15){
                    setHSlidePos(TRAY_HSLIDE_INI);
                    waitTime = 700;
                }

                trayRotatePos = 0;
                resetHSlide();
                setTrayRotateAngle(ROTATE_INIT_ANGLE);

                armTiltAngle = ARM_TILT_INI_ANGLE;
                setArmTiltAngle(armTiltAngle);
                trayTiltAngle = TRAY_TILT_INI_ANGLE;
                setTrayTargetAngle(trayTiltAngle);
                mainTask.sleep(waitTime);

                trayOpenBothLock();

                double checkLen = RESET_LM_LEN;
                if (getSliderLen() < 0.2){
                    checkLen = 0.2;
                }
                sliderLenCtrl(checkLen,1.0);



                runTime.reset();
                while(mainTask.opModeIsActive() && !mainTask.gamepad2.back && Math.abs(getSliderLen() - checkLen) > 0.01 && runTime.milliseconds() < 3000){
                    mainTask.sleep(10);
                    if (runTime.milliseconds() > 1000) break;
                }



                runTime.reset();
                if (!mainTask.gamepad2.back && runTime.milliseconds() < 900) {
                    //setTrayTargetAngle(TRAY_TILT_INI_ANGLE);

                    armTiltAngle = ARM_TILT_INI_ANGLE;
                    setArmTiltAngle(armTiltAngle);
                    trayOpenBothLock();
                    //mainTask.sleep(350);
                    resetSlider();
                    runTime.reset();
                    while (mainTask.opModeIsActive() && !mainTask.gamepad2.back && getSliderLen() > 0.03 && runTime.milliseconds() < 3000 ) {
                        mainTask.sleep(10);
                        if (runTime.milliseconds() > 1500){
                            break;
                        }
                    }
                }
                dropLevel = -1;
                movingFlag = false;

            }
        }).start();

    }


    public void toDropLevelPos(int level){
        new Thread(new Runnable() {
            @Override
            public void run() {
                movingFlag = true;
                intakeMotor.setPower(0);

                trayCloseBothLock();
                mainTask.sleep(200);  // suppose already lock the pixed
                //slide move first
                sliderLenCtrl(LEVEL_LEN[level], 1.0);
                runTime.reset();
                while(mainTask.opModeIsActive() && !mainTask.gamepad2.back && getSliderLen() < RESET_LM_LEN && runTime.milliseconds() < 3000){
                    mainTask.sleep(10);
                    if (runTime.milliseconds() > 2000) {
                        break;
                    }
                }


                if (!mainTask.gamepad2.back && runTime.milliseconds() < 2000) {
                    if (dropLevel < 0){
                        trayRotatePos = 1;
                        if (Math.abs(getTrayRoateAngle()) < 10) {
                            setTrayRotateAngle(ROTATE_LEFT_ANGLE);
                        }
                    }

                    armTiltAngle = LEVEL_ARM_ANGLE[level];
                    setArmTiltAngle(armTiltAngle);
                    trayCloseBothLock();
                    trayTiltAngle = LEVEL_TRAY_ANGLE[level];
                    setTrayTiltAngle(trayTiltAngle);
                    runTime.reset();
                    while(mainTask.opModeIsActive() && !mainTask.gamepad2.back  && runTime.milliseconds() < 500){
                        mainTask.sleep(10);
                    }

                }
                mainTask.sleep(100);

                dropLevel = level;
                movingFlag = false;


            }
        }).start();
    }

    public void autoToDropLevelPos(int level, double rotateAngle){
        new Thread(new Runnable() {
            @Override
            public void run() {
                movingFlag = true;
                trayCloseBothLock();
                mainTask.sleep(400);
                intakeMotor.setPower(0);
                //slide move first
                sliderLenCtrl(AUTO_LEVEL_LEN[level], 1.0);
                runTime.reset();
                while(mainTask.opModeIsActive() && getSliderLen() < AUTO_LEVEL_LEN[level] - 0.02 && !mainTask.gamepad2.back && runTime.milliseconds() < 3000){
                    mainTask.sleep(10);
                    if (runTime.milliseconds() > 2000) {
                        break;
                    }
                }
                if (!mainTask.gamepad2.back && runTime.milliseconds() < 2000) {
                    trayRotatePos = 1;
                    setTrayRotateAngle(rotateAngle);

                    armTiltAngle = AUTO_LEVEL_ARM_ANGLE[level];
                    setArmTiltAngle(armTiltAngle);
                    trayTiltAngle = AUTO_LEVEL_TRAY_ANGLE[level];
                    setTrayTiltAngle(trayTiltAngle);
                    runTime.reset();
                    while(mainTask.opModeIsActive() && !mainTask.gamepad2.back  && runTime.milliseconds() < 500){
                        mainTask.sleep(10);
                    }

                }
                mainTask.sleep(100);

                dropLevel = level;
                movingFlag = false;


            }
        }).start();
    }

    public void trayOpenBothLock(){

        frontLockServo.setPosition(TRAY_OPEN_POS);
        backLockServo.setPosition(TRAY_OPEN_POS);
        if (!auto){
            new Thread(new Runnable() {
                @Override
                public void run() {
                    mainTask.sleep(200);
                    backLockState(false);
                    frontLockState(false);
                }
            }).start();

        }
    }

    public void trayLockHoldPos(){

        frontLockServo.setPosition(TRAY_HOLD_POS);
        backLockServo.setPosition(TRAY_HOLD_POS);
    }

    public void trayCloseBothLock(){

        frontLockServo.setPosition(TRAY_CLOSE_POS);
        backLockServo.setPosition(TRAY_CLOSE_POS);
        if (!auto){
            new Thread(new Runnable() {
                @Override
                public void run() {
                    mainTask.sleep(200);
                    backLockState(true);
                    frontLockState(true);
                }
            }).start();

        }
    }

    public void trayOpenFrontLock(){
        frontLockServo.setPosition(TRAY_OPEN_POS);
        if (!auto){
            new Thread(new Runnable() {
                @Override
                public void run() {
                    mainTask.sleep(200);
                    frontLockState(false);
                }
            }).start();

        }
    }


    public void trayOpenBackLock(){
        backLockServo.setPosition(TRAY_OPEN_POS);
        if (!auto){
            new Thread(new Runnable() {
                @Override
                public void run() {
                    mainTask.sleep(200);
                    backLockState(false);
                }
            }).start();

        }
    }







    public void disableTrayServo(){
        trayTiltServo.getController().pwmDisable();
    }
    public void enableTrayServo(){
        trayTiltServo.getController().pwmEnable();
    }



    public void setToIntakePos(){
        trayOpenBothLock();
    }

    public void forceResetSlide(){
        new Thread(new Runnable() {
            @Override
            public void run() {
                trayOpenBothLock();
                resetHSlide();
                mainTask.sleep(500);
                sliderLenCtrl(0.2,1.0);

                mainTask.sleep(500);


                armTiltAngle = ARM_TILT_INI_ANGLE;
                setArmTiltAngle(armTiltAngle);

                trayTiltAngle = TRAY_TILT_INI_ANGLE;
                setTrayTargetAngle(trayTiltAngle);
                trayOpenBothLock();
                setTrayRotateAngle(ROTATE_INIT_ANGLE);
                dropLevel = -1;
                mainTask.sleep(300);

                sliderMtr1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                // sliderMtr2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sliderMtr1.setPower(-0.5);
                //mainTask.sleep(500);
                //sliderMtr2.setPower(-0.2);
                runTime.reset();
                while (!mainTask.isStopRequested() && !resetTouch.isPressed() && runTime.milliseconds() < 2000) {
                    mainTask.sleep(10);
                    mainTask.telemetry.addLine("not touched");
                    mainTask.telemetry.update();
                }

                sliderMtr1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                // sliderMtr2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                mainTask.sleep(200);
                sliderLen = getSliderLen();


            }
        }).start();
    }


    public void initLed(){
        frontRedLed = mainTask.hardwareMap.get(DigitalChannel.class, "red2");
        frontGreenLed = mainTask.hardwareMap.get(DigitalChannel.class, "green2");

        backRedLed = mainTask.hardwareMap.get(DigitalChannel.class, "red1");
        backGreenLed = mainTask.hardwareMap.get(DigitalChannel.class, "green1");

        frontRedLed.setMode(DigitalChannel.Mode.OUTPUT);
        frontGreenLed.setMode(DigitalChannel.Mode.OUTPUT);

        backRedLed.setMode(DigitalChannel.Mode.OUTPUT);
        backGreenLed.setMode(DigitalChannel.Mode.OUTPUT);

        frontRedLed.setState(false);
        frontGreenLed.setState(true);

        backRedLed.setState(false);
        backGreenLed.setState(true);
    }

    public void frontLockState(boolean lock){
        if (lock){
            frontRedLed.setState(true);
            frontGreenLed.setState(false);
        }
        else{
            frontRedLed.setState(false);
            frontGreenLed.setState(true);
        }
    }
    public void backLockState(boolean lock){
        if (lock){
            backRedLed.setState(true);
            backGreenLed.setState(false);
        }
        else{
            backRedLed.setState(false);
            backGreenLed.setState(true);
        }
    }

    public void resetTrayRotate(){
        trayRotatePos = 0;
        trayRotateAngle = 0;
        setTrayRotateAngle(trayRotateAngle);
    }


  }
