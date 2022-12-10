package org.firstinspires.ftc.teamcode.Encoders;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
@Autonomous (name = "Color")
public class Color extends LinearOpMode {

    ColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {
        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("red: ", colorSensor.red());
            telemetry.addData("green: ", colorSensor.green());
            telemetry.addData("blue: ", colorSensor.blue());
            telemetry.addData("alpha: ", colorSensor.alpha());
            telemetry.update();
        }
    }
}
