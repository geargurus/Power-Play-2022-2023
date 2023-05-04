package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class Proportionaltears extends OpMode {
    private PIDController controller;
    //proportional, integral, derivative
    public static double p = 0, i = 0, d = 0;
    //feedforward
    public static double f = 0;

    public static int target = 0;

    private final double MathTicks    = 537.7 / 180.0 ;

    private DcMotor crAzyTaxi;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        crAzyTaxi = hardwareMap.get(DcMotor.class, "crAzyTaxi");
    }

    @Override
    public  void loop() {
        controller.setPID(p, i, d);
        int armPos = crAzyTaxi.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / MathTicks)) * f;

        double power = pid + ff;

        crAzyTaxi.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}