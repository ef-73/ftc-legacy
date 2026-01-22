package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;
@TeleOp(name="E_testing", group="Linear Opmode")
//@Disabled
//
public class E_Phase1TeleOpPowerPlay extends LinearOpMode{

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor lf, rf, lr, rr, intake, outtake;
    Servo claw1L, claw1R, pan1, tilt1, claw2L, claw2R, pan2, tilt2;
    TouchSensor touch_in, touch_out;
    double intake_out = 1610, intake_in = 250;

    double disM_encoderHtoCenter = 0.055;// distance from horizontal odom wheel to the center of the robot
    //0.065M for goBilda odom wheel bot || 0.055M for openodometry wheel bot
    BNO055IMU               imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle;


    private NormalizedColorSensor colorSensor = null;


    @Override
    public void runOpMode() {
        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        lf = hardwareMap.get(DcMotor.class, "lf");
        lr = hardwareMap.get(DcMotor.class, "lr");
        rf = hardwareMap.get(DcMotor.class, "rf");
        rr = hardwareMap.get(DcMotor.class, "rr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        outtake = hardwareMap.get(DcMotor.class, "outtake");

        pan1 = hardwareMap.get(Servo.class, "pan1");
        tilt1 = hardwareMap.get(Servo.class, "tilt1");
        claw1L = hardwareMap.get(Servo.class, "claw1L");
        claw1R = hardwareMap.get(Servo.class, "claw1R");

        pan2 = hardwareMap.get(Servo.class, "pan2");
        tilt2 = hardwareMap.get(Servo.class, "tilt2");
        claw2L = hardwareMap.get(Servo.class, "claw2L");
        claw2R = hardwareMap.get(Servo.class, "claw2R");

        claw1R.setDirection(Servo.Direction.REVERSE);
        claw2R.setDirection(Servo.Direction.REVERSE);

        touch_in = hardwareMap.touchSensor.get("touch1");//touch sensor for intake
        touch_out = hardwareMap.touchSensor.get("touch2");//touch sensor for outtake

        //colorSensor = hardwareMap.get(NormalizedColorSensor.class, "base_color");
        //arm = hardwareMap.get(DcMotor.class, "arm");
        //extend = hardwareMap.get(DcMotor.class, "lift");

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        intake.setDirection(DcMotor.Direction.REVERSE); //+: reach out ; -: reach in
        outtake.setDirection(DcMotor.Direction.REVERSE); //+: up ; -: dowm
        lf.setDirection(DcMotor.Direction.REVERSE);
        lr.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        rr.setDirection(DcMotor.Direction.FORWARD);
        //arm.setDirection(DcMotorSimple.Direction.REVERSE);

        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        outtake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Orientation lastAngles = new Orientation();
        double globalAngle;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");

        waitForStart();
        imu.initialize(parameters);

        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");


        telemetry.update();
        //reset_arms();
        reset_claws();
        while (opModeIsActive()) {


            if (gamepad1.dpad_up && intake.getCurrentPosition()<=1500){
                intake.setPower(0.5);
            }
            else if (gamepad1.dpad_down && intake.getCurrentPosition()>10){
                intake.setPower(-0.5);
            }
            else{
                intake.setPower(0);
            }
            if (gamepad1.dpad_left && outtake.getCurrentPosition()<=1615){
                outtake.setPower(0.5);
            }
            else if (gamepad1.dpad_right && outtake.getCurrentPosition()>20){
                outtake.setPower(-0.5);
            }
            else{
                outtake.setPower(0);
            }
            if (gamepad1.a){
                claw1_catch();
            }
            if (gamepad1.b){
                claw1_release();
            }
            if (gamepad1.x){
                claw2_catch();
            }
            if (gamepad1.y){
                claw2_release();
            }
            if (gamepad1.left_bumper){
                arm_to_pos(1600, false);
            }
            telemetry.addData("intake_encoder", intake.getCurrentPosition());
            telemetry.addData("outtake_encoder", outtake.getCurrentPosition());
            telemetry.update();
        }
    }
    void reset_arms(){

        while (!touch_in.isPressed()){
            intake.setPower(-0.1);
        }
        intake.setPower(0);
        while(!touch_out.isPressed()){
            outtake.setPower(-0.1);
        }
        outtake.setPower(0);
        outtake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    void reset_claws(){
        tilt1.setPosition(0.5);
        pan1.setPosition(0.5);
        claw1_close();
        tilt2.setPosition(0.5);
        pan2.setPosition(0.5);
        claw2_close();
    }
    void claw1_open(){
        claw1L.setPosition(0.3);
        claw1R.setPosition(0.3);
    }
    void claw1_close(){
        double claw1_target = 0.13;
        double claw1L_last = claw1L.getPosition();
        double claw1R_last = claw1R.getPosition();
        claw1L.setPosition(claw1_target);
        claw1R.setPosition(claw1_target);

        while (claw1L.getPosition()>claw1_target && claw1R.getPosition()>claw1_target){
            double claw1L_delta = claw1L_last - claw1L.getPosition();
            double claw1R_delta = claw1R_last - claw1R.getPosition();
            if (claw1L_delta < 0.1 && claw1R_delta < 0.1){
                claw1L.setPosition(claw1L.getPosition());
                claw1R.setPosition(claw1R.getPosition());
                telemetry.addData("claw1_close", "finished");
                telemetry.update();
                break;
            }
            claw1L_last = claw1L.getPosition();
            claw1R_last = claw1R.getPosition();
        }
    }
    void claw1_catch(){
        pan1.setPosition(0.5);
        claw1_close();
        arm_to_pos(intake_out, true);
        tilt1.setPosition(0.16);
        sleep(500);
        claw1_open();
    }
    void claw1_release(){
        claw1_close();
        arm_to_pos(intake_in, true);
        sleep(500);
        tilt1.setPosition(0.65);
    }
    void claw2_open(){
        claw2L.setPosition(0.3);
        claw2R.setPosition(0.3);
    }
    void claw2_close(){
        claw2L.setPosition(0.5);
        claw2R.setPosition(0.5);
    }
    void claw2_catch(){
        pan2.setPosition(0.5);
        tilt2.setPosition(0.96);
        claw2_open();
    }
    void claw2_release(){

        claw2_close();
        sleep(500);
        claw1_open();
        tilt2.setPosition(0.3);
        sleep(1500);
        claw2_open();
    }
    void arm_to_pos(double target_encoder, boolean in){
        DcMotor arm;
        if (target_encoder<0){
            target_encoder = 0;
        }
        if (target_encoder>1610){
            target_encoder = 1610;
        }
        if (in){
            arm = intake;
        }
        else{
            arm = outtake;
        }
        double delta = target_encoder - arm.getCurrentPosition();
        while (Math.abs(delta)>50){
            arm.setPower(0.5*delta);
            delta = target_encoder - arm.getCurrentPosition();
            if (Math.abs(delta)<20 || gamepad1.a || arm.getCurrentPosition()>1600){
                arm.setPower(0.01);
                break;
            }
        }
    }
    private class intake_thread extends Thread{
        boolean pickup;
        public intake_thread(boolean pickup){
            this.pickup = pickup;
        }
        public void run(){
            try{

            }
            catch(Exception e){}
        }
    }
}