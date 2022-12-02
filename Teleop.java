package org.firstinspires.ftc.teamcode.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Teleop")
public class Teleop extends OpMode {

    DcMotor frontLeft;
    DcMotor backLeft;
    DcMotor frontRight;
    DcMotor backRight;
    DcMotor Cascade;
    CRServo inTake;

    @Override
    public void init() {
        frontLeft = hardwareMap.dcMotor.get("frontLeft");
        backLeft = hardwareMap.dcMotor.get("backLeft");
        frontRight = hardwareMap.dcMotor.get("frontRight");
        backRight = hardwareMap.dcMotor.get("backRight");
        Cascade = hardwareMap.dcMotor.get("Cascade");
        inTake = hardwareMap.crservo.get("inTake");
    }

    @Override
    public void loop() {
        if (Math.abs(-gamepad1.left_stick_y) > .1) {
            frontLeft.setPower(-gamepad1.left_stick_y * -.6);
            backLeft.setPower(-gamepad1.left_stick_y * .6);
        } else {
            frontLeft.setPower(0);
            backLeft.setPower(0);
        }
        if (Math.abs(-gamepad1.right_stick_y) > .1) {
            frontRight.setPower(-gamepad1.right_stick_y * -.6);
            backRight.setPower(-gamepad1.right_stick_y * .6);
        } else {
            frontRight.setPower(0);
            backRight.setPower(0);
        }
        if (gamepad2.right_bumper) {
            inTake.setPower(.5);
        }
        else {
            inTake.setPower(0);
        }
        if (gamepad2.left_bumper){
            inTake.setPower(-.5);
        }
        else {
            inTake.setPower(0);
        }
        if (Math.abs(-gamepad1.right_trigger) > .1) {
            frontLeft.setPower(.6);
            backLeft.setPower(-.6);
            frontRight.setPower(.6);
            backRight.setPower(-.6);
        }
        else {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
        if (Math.abs(-gamepad1.left_trigger) > .1) {
            frontLeft.setPower(-6);
            backLeft.setPower(.6);
            frontRight.setPower(-.6);
            backRight.setPower(.6);
        }
        else {
            frontLeft.setPower(0);
            backLeft.setPower(0);
            frontRight.setPower(0);
            backRight.setPower(0);
        }
        if (Math.abs(-gamepad2.left_stick_y) > .1) {
            Cascade.setPower(-gamepad2.left_stick_y * .5);
        }
        else {
            Cascade.setPower(0);
        }
    }
}