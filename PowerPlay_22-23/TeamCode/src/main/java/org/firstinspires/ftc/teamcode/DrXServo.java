package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class DrXServo {
    LinearOpMode mainTask;
    private Servo servo1 = null;
    private Servo servo2 = null;
    private double degree_to_pos_ratio = 1.0 / 270;
    /*
    private double                  Degree_to_Servo_Gobilda_value_ratio = 1.0/270;
    private double                  Degree_to_Servo_Feetech360_Value_Ratio = 0.278/90;
    private double                  Degree_to_Servo_Feetech180F_Value_Ratio = 0.58/90;
    private double                  Degree_to_Servo5Turn_value_Ratio = 0.1129245/180;

     */
    private String servoName;
    private double ZERO_POS = 0.5;
    private double zero_offset = 0;
    private double targetAngle;
    private double targetPos;
    private boolean servo_plan_ctrl = false;
    private double maxStep = 2.52;
    private double planStep = maxStep;
    private final long CTRL_PERIOD = 10;
    private boolean firstFlag = true;

    private double taskTime = 0;

    public DrXServo(LinearOpMode mainTask, String deviceName, double degree_to_pos_ratio, double maxSpeedDegreeInmSec, double zero_pos) {
        this.mainTask = mainTask;
        this.servo1 = mainTask.hardwareMap.get(Servo.class, deviceName);
        this.servo2 = null;
        this.servoName = deviceName;
        this.degree_to_pos_ratio = degree_to_pos_ratio;
        this.maxStep = maxSpeedDegreeInmSec * this.degree_to_pos_ratio * CTRL_PERIOD;  // now is servo position value , not degree
        ZERO_POS = zero_pos;
        servo_plan_ctrl = false;
        Servo_Pos_Run_Thread_Task();
        firstFlag = true;

    }

    public DrXServo(LinearOpMode mainTask, String deviceName1, String deviceName2, double degree_to_pos_ratio, double maxSpeedDegreeInmSec, double zero_pos) {
        this.mainTask = mainTask;
        this.servo1 = mainTask.hardwareMap.get(Servo.class, deviceName1);
        this.servo2 = mainTask.hardwareMap.get(Servo.class, deviceName2);
        this.servoName = deviceName1;
        this.degree_to_pos_ratio = degree_to_pos_ratio;
        this.maxStep = maxSpeedDegreeInmSec * this.degree_to_pos_ratio * CTRL_PERIOD;  // now is servo position value , not degree
        ZERO_POS = zero_pos;
        servo_plan_ctrl = false;
        Servo_Pos_Run_Thread_Task();
        firstFlag = true;

    }

    public void setDirection(Servo.Direction direction){
        this.servo1.setDirection(direction);
    }

    public void setDirection(Servo.Direction direction1, Servo.Direction direction2){
        this.servo1.setDirection(direction1);
        if (this.servo2 != null) servo2.setDirection(direction2);
    }

    //below 2 functions just for keeping same functions as normal Servo class, do not use them, so we set them as private
    private void setPosition(double position){
        servo_plan_ctrl = false;
        this.servo1.setPosition(position);
        if (this.servo2 != null){
            this.servo2.setPosition(position);
        }
    }

    private double getPosition(){
        return this.servo1.getPosition();
    }

    public void setTargetAngle(double angle) {

        targetAngle = angle;
        targetPos = angle * degree_to_pos_ratio + ZERO_POS;
        targetPos = targetPos > 1.0 ? 1.0 : targetPos;
        targetPos = targetPos < 0 ? 0 : targetPos;
        servo_plan_ctrl = true;
        planStep = targetPos > servo1.getPosition() ? maxStep : -maxStep;
        taskTime = Math.abs(targetPos - servo1.getPosition()) / maxStep * CTRL_PERIOD;
        // mainTask.telemetry.addData(servoName + " target", "%3.2f", targetPos);
       // mainTask.telemetry.update();

    }

    public void setTargetAngle(double angle, double time) {

        targetAngle = angle;
        targetPos = angle * degree_to_pos_ratio + ZERO_POS;
        servo_plan_ctrl = true;
        planStep = (targetPos - servo1.getPosition()) / (time / CTRL_PERIOD);
        planStep = Math.abs(planStep) > maxStep ? Math.signum(planStep) * maxStep : planStep;
        taskTime = time;
    }

    public double getCurrentAngle(){
        return (servo1.getPosition() - ZERO_POS) / degree_to_pos_ratio;
    }

    public double getTaskTim(){
        return taskTime;
    }

    public void disablePosPlanCtrl(){
        servo_plan_ctrl = false;
    }

    void Servo_Pos_Run_Thread_Task() {
        new Thread(new Runnable() {
            @Override

            public void run() {
                double currentPos = 0;
                double remain = 0;
                while (!mainTask.isStopRequested() ) {  //&& mainTask.opModeIsActive()
                    if (servo_plan_ctrl) {
                        if (firstFlag){
                            servo1.setPosition(targetPos);
                            if (servo2 != null) servo2.setPosition(targetPos);
                            firstFlag = false;
                        }
                        else{
                            currentPos = servo1.getPosition();
                            remain = targetPos - currentPos;
                            if (Math.abs(remain) > Math.abs(planStep) && !firstFlag) {
                                servo1.setPosition(currentPos + planStep);
                                if (servo2 != null) servo2.setPosition(currentPos + planStep);
                            }
                            else{
                                servo1.setPosition(targetPos);
                                if (servo2 != null) servo2.setPosition(targetPos);
                            }
                        }
/*
                        if (firstFlag){
                            mainTask.telemetry.addLine("First");
                        }
                        else{
                            mainTask.telemetry.addLine("not First");
                        }
                        mainTask.telemetry.addData(servoName + " servo", "%3.2f", currentPos);
                        mainTask.telemetry.addData(servoName + " remain servo", "%3.2f", remain);

                        mainTask.telemetry.update();
*/
                    }
                    mainTask.sleep(CTRL_PERIOD);
                }

            }
        }).start();

    }
}