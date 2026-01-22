package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class HangArmControl {
    private LinearOpMode mainTask;
    public DcMotorEx hangMotor = null;

    private final int HANG_UP_POS = 1900;
    private final int HANG_DOWN_POS = 420;
    private boolean hangMotorMoving = false;


    HangArmControl(LinearOpMode mainTask){
        this.mainTask = mainTask;

        hangMotor = mainTask.hardwareMap.get(DcMotorEx.class, "hangMotor");
        hangMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        hangMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        hangMotor.setPower(0);
    }


    public void hangArmUp(){
        if (!hangMotorMoving) {
            new Thread(new Runnable() {

                @Override
                public void run() {
                    hangMotorMoving = true;
                    hangMotor.setTargetPosition(HANG_UP_POS);
                    hangMotor.setPower(1.0);
                    hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (mainTask.opModeIsActive() && Math.abs(hangMotor.getCurrentPosition() - HANG_UP_POS) > 100) {
                        mainTask.sleep(10);
                    }
                    mainTask.sleep(200);
                    hangMotor.setTargetPosition(HANG_DOWN_POS);
                    hangMotor.setPower(1.0);
                    hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    while (mainTask.opModeIsActive() && Math.abs(hangMotor.getCurrentPosition() - HANG_DOWN_POS) > 100) {
                        mainTask.sleep(10);
                    }
                    hangMotorMoving = false;
                }
            }).start();
        }


    }

    public void hangArmDown(){
        hangMotor.setTargetPosition(HANG_DOWN_POS);
        hangMotor.setPower(1.0);
        hangMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

}
