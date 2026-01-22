package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class AirPlaneLaunch {
    private DcMotorEx airplaneLaunchMotor = null;
    private Servo airPlaneLaunchServo = null;
   // private Servo openCoverServo = null;

    private final double SERVO_RESET_POS = 0.5;
    private final double SERVO_LAUNCH_POS = 1.0 ;// 1.0 for continuous,  servo 0.8 for servo;

    private final double MOTOR_LAUNCH_POWER = 1.0;

    public final double COVER_RESET_POS = 0.495;

    public final double COVER_LAUNCH_POS = 0.22;

    public boolean airplaneMotorStart = false;
    private boolean swingCoverServo = false;

    private LinearOpMode mainTask;

    AirPlaneLaunch(LinearOpMode mainTask){
        this.mainTask = mainTask;

        airplaneLaunchMotor = mainTask.hardwareMap.get(DcMotorEx.class, "airplaneLaunchMotor");
        airplaneLaunchMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        airplaneLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        airplaneLaunchMotor.setPower(0);

        airPlaneLaunchServo = mainTask.hardwareMap.get(Servo.class, "airplaneLaunchServo");
        airPlaneLaunchServo.setDirection(Servo.Direction.FORWARD);
        airPlaneLaunchServo.setPosition(SERVO_RESET_POS);

     //   openCoverServo = mainTask.hardwareMap.get(Servo.class, "openCoverServo");
    //    openCoverServo.setDirection(Servo.Direction.FORWARD);
    //    openCoverServo.setPosition(COVER_RESET_POS);
        airplaneMotorStart = false;
        swingCoverServo = false;
    }


    // in the thread ???
    public void launchAirPlane(){
        if (!airplaneMotorStart) {


            new Thread(new Runnable() {
                @Override
                public void run() {
        //            openCoverServo.setPosition(COVER_LAUNCH_POS);
                    airplaneLaunchMotor.setPower(MOTOR_LAUNCH_POWER);
                    mainTask.sleep(1500);
                    airplaneMotorStart = true;
                    airPlaneLaunchServo.setPosition(SERVO_LAUNCH_POS);
                    mainTask.sleep(1500);
                    airplaneLaunchMotor.setPower(0);
                    airplaneMotorStart = false;
                    airPlaneLaunchServo.setPosition(SERVO_RESET_POS);
                }
            }).start();

        }
        else{
            new Thread(new Runnable() {
                @Override
                public void run() {
                  //  openCoverServo.setPosition(COVER_LAUNCH_POS);
                   // mainTask.sleep(600);
                    airPlaneLaunchServo.setPosition(SERVO_LAUNCH_POS);
                    mainTask.sleep(1500);
                    airplaneLaunchMotor.setPower(0);
                    airplaneMotorStart = false;
                    airPlaneLaunchServo.setPosition(SERVO_RESET_POS);
                }
            }).start();
        }
    }


    public void swingCoverServo(){
        if (!swingCoverServo) {

            new Thread(new Runnable() {
                @Override
                public void run() {
                    swingCoverServo = true;
         //           openCoverServo.setPosition(0.8);
                    mainTask.sleep(800);
        //            openCoverServo.setPosition(0.22);
                    mainTask.sleep(800);
        //            openCoverServo.setPosition(COVER_RESET_POS);
                    swingCoverServo = false;
                }
            }).start();

        }


    }

    public void startAirplaneMotor(){
        airplaneMotorStart = true;
        airplaneLaunchMotor.setPower(MOTOR_LAUNCH_POWER);
   //     openCoverServo.setPosition(COVER_LAUNCH_POS);
    }

    public void stopAirplaneMotor(){
        airplaneMotorStart = false;
        airplaneLaunchMotor.setPower(0);
    //    openCoverServo.setPosition(COVER_RESET_POS);

    }





}
