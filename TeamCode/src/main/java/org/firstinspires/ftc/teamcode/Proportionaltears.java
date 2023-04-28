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

    private final double COUNTS_PER_MOTOR_REV    = 1440 ;

    private DcMotor outSlide;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outSlide = hardwareMap.get(DcMotor.class, "outSlide");
    }

    @Override
    public  void loop() {
        controller.setPID(p, i, d);
        int armPos = outSlide.getCurrentPosition();
        double pid = controller.calculate(armPos, target);
        double ff = Math.cos(Math.toRadians(target / COUNTS_PER_MOTOR_REV)) * f;

        double power = pid + ff;

        outSlide.setPower(power);

        telemetry.addData("pos", armPos);
        telemetry.addData("target", target);
        telemetry.update();
    }
}
