package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Ethan_longlegs_opmode")
public class Daddy_Longlegs_opmode extends LinearOpMode {

    
    @Override
    public void runOpMode() {
        Daddy_Longlegs_config robot = new Daddy_Longlegs_config();

        robot.init(hardwareMap);
        //robot.arm_tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);//hold the arm up
        telemetry.addLine("done calibration, ready to start");
        telemetry.update();
        waitForStart();
        float clawPos = 0f;
        float basketMovePosition = 0.5f;
        while (opModeIsActive()) {

            int tilt_position = robot.arm_tilt.getCurrentPosition();
            telemetry.addData("Encoder Position", tilt_position);

            int stretch_position = robot.arm_stretch.getCurrentPosition();
            telemetry.addData("Encoder Position", stretch_position);
            float leftStickY = -this.gamepad1.left_stick_y;
            float leftStickX = this.gamepad1.left_stick_x;
            float rightStickX = this.gamepad1.right_stick_x;

            boolean intakeBtn = this.gamepad1.left_bumper;
            boolean outakeBtn = this.gamepad1.right_bumper;
            boolean armDown = this.gamepad1.a;
            boolean armMedium = this.gamepad1.b;
            boolean armUp = this.gamepad1.y;

            boolean armExtend = this.gamepad1.dpad_up;
            boolean armRetract = this.gamepad1.dpad_down;
            boolean spin = this.gamepad1.x;
            boolean basketLeft = this.gamepad1.dpad_left;


            robot.leftFrontDrive.setPower(leftStickX + leftStickY + rightStickX);
            robot.rightFrontDrive.setPower(-leftStickX + leftStickY - rightStickX);
            robot.leftRearDrive.setPower(-leftStickX + leftStickY + rightStickX);
            robot.rightRearDrive.setPower(leftStickX + leftStickY - rightStickX);
/*
            robot.claw_rotate.setPosition(basketTiltPosition);
            robot.claw_move.setPosition(basketMovePosition);
           */
            if (intakeBtn) {
                robot.claw.setPosition(1);

            } else if (outakeBtn) {
                robot.claw.setPosition(-1);

            } else {

                robot.claw.setPosition(robot.claw.getPosition());
            }

            if (armUp) {
                robot.arm_tilt.setPower(0.5);
            } else if (armMedium) {
                robot.arm_tilt.setPower(-0.5);
            } else
                robot.arm_tilt.setPower(0);
            }
//            if (armExtend) {
//                robot.arm_stretch.setPower(0.5);
//            } else if (armRetract) {
//                robot.arm_stretch.setPower(-0.5);
//            } else {
//                robot.arm_stretch.setPower(0);
//            }
//            if (spin) {
//                robot.spinner.setPower(1.0);
//            }else {
//                robot.spinner.setPower(0);
//            }
            /*
            if (basketUp) {
                basketTiltPosition += 0.05f;
            } else if (basketDown) {
                basketTiltPosition -= 0.05f;
            } else {
                basketTiltPosition += 0;
            }
            if (basketRight) {
                basketMovePosition += 0.05f;
            } else if (basketLeft) {
                basketMovePosition -= 0.05f;
            } else {
                basketMovePosition += 0;
            }

*/

        }
    }

