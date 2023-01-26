package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous (name = "EncoderWheelTest")
public class EncoderWheelTest extends LinearOpMode {

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    @Override
    public void runOpMode() throws InterruptedException {

        fr = hardwareMap.get(DcMotor.class, "frontRight");
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");

    }
}
