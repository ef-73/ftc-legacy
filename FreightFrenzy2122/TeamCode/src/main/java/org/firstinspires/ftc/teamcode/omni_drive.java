
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp(name = "omni drive")
public class omni_drive extends LinearOpMode {
    @Override
    public void runOpMode(){
        robot_hardware robot = new robot_hardware();
        robot.init(hardwareMap,telemetry);
        robot.lf_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        robot.lr_drive.setDirection(DcMotorSimple.Direction.REVERSE);
        telemetry.addLine("done calibration");
        telemetry.update();
        waitForStart();
        while(opModeIsActive()){
            float leftsticky = -this.gamepad1.left_stick_y;
            float leftstickx = this.gamepad1.left_stick_x;
            float rightstickx = this.gamepad1.right_stick_x;
            boolean intakeBtn = this.gamepad1.left_bumper;
            boolean outakeBtn = this.gamepad1.right_bumper;
            boolean armup = this.gamepad1.a;
            boolean armdown = this.gamepad1.b;
            robot.lf_drive.setPower(leftstickx+leftsticky+rightstickx);
            robot.rf_drive.setPower(-leftstickx+leftsticky-rightstickx);
            robot.lr_drive.setPower(-leftstickx+leftsticky+rightstickx);
            robot.rr_drive.setPower(leftstickx+leftsticky-rightstickx);
            if(intakeBtn) {
                robot.right_intake.setPower(1);
                robot.left_intake.setPower(-1);
            }else if(outakeBtn){
                robot.right_intake.setPower(-1);
                robot.left_intake.setPower(1);
            }else{
                robot.right_intake.setPower(0);
                robot.left_intake.setPower(0);
            }
            if(armup){
                robot.arm.setPower(-1);
            }else if(armdown){
                robot.arm.setPower(1);
            }else{
                robot.arm.setPower(0);
            }
        }
    }
}