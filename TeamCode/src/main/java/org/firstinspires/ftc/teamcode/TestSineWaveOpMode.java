package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TestSineWaveOpMode extends OpMode {
    FtcDashboard dashboard;
    public static double AMPLITUDE = 1;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.25;


    public TestSineWaveOpMode(){
        super("TestSineWaveOpMode");
    }

    @Override
    protected void init() {
        dashboard = FtcDashboard.getInstance();
    }

    @Override
    protected void loop() throws InterruptedException {
        System.out.println(Math.sin(System.currentTimeMillis()));
        Telemetry dashboardTelemetry = dashboard.getTelemetry();

        dashboardTelemetry.addData("x", AMPLITUDE * Math.sin(
                2 * Math.PI * FREQUENCY * (System.currentTimeMillis() / 1000d) + Math.toRadians(PHASE)
        ));
        dashboardTelemetry.update();
        Thread.sleep(1);
    }
}