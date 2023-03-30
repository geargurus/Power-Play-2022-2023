package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous (name = "EncoderWheelTest")
public class EncoderWheelTest extends LinearOpMode {

    private DcMotor fl;
    private DcMotor fr;
    private DcMotor bl;
    private DcMotor br;

    private ElapsedTime runtime = new ElapsedTime();

    static final double COUNTS_PER_MOTOR_REV = 538;
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.7;
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.4;
    @Override
    public void runOpMode() throws InterruptedException {

        fr = hardwareMap.get(DcMotor.class, "frontRight");
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        br = hardwareMap.get(DcMotor.class, "backRight");
        bl = hardwareMap.get(DcMotor.class, "backLeft");

        waitForStart();
        //telemetry.addData("Counts:", COUNTS_PER_MOTOR_REV);
        encoderFLeft(DRIVE_SPEED, 20.0, 300.0);
        encoderFRight(DRIVE_SPEED, 20.0, 300);
        encoderbLeft(DRIVE_SPEED, 20.0, 300);
        encoderbRight(DRIVE_SPEED, 20.0, 300);

    }

    private void encoderFLeft(double speed, double inches, double timeoutS) {
        int newfltarget;

        if (opModeIsActive()) {
            newfltarget = (fl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
            fl.setTargetPosition(newfltarget);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            fl.setPower(Math.abs(speed));

            while (opModeIsActive()&& fl.isBusy() && runtime.seconds() < timeoutS){
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d ", newfltarget);
                telemetry.addData("Currently at",  " at %7d", fl.getCurrentPosition());
                telemetry.update();
            }
            //code here
            fl.setPower(0);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
        }
        private void encoderFRight(double speed, double inches, double timeoutS) {
        int newfrtarget;

        if (opModeIsActive()) {
            newfrtarget = (fr.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
            fr.setTargetPosition(newfrtarget);
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            fr.setPower(Math.abs(speed));

            while (opModeIsActive()&& fr.isBusy() && runtime.seconds() < timeoutS){
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d ", newfrtarget);
                telemetry.addData("Currently at",  " at %7d", fr.getCurrentPosition());
                telemetry.update();
            }
            //code here
            fr.setPower(0);
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        }
    private void encoderbLeft(double speed, double inches, double timeoutS) {
        int newbltarget;

        if (opModeIsActive()) {
            newbltarget = (bl.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
            bl.setTargetPosition(newbltarget);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            bl.setPower(Math.abs(speed));

            while (opModeIsActive()&& bl.isBusy() && runtime.seconds() < timeoutS){
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d ", newbltarget);
                telemetry.addData("Currently at",  " at %7d", bl.getCurrentPosition());
                telemetry.update();
            }
            //code here
            bl.setPower(0);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    private void encoderbRight(double speed, double inches, double timeoutS) {
        int newbrtarget;

        if (opModeIsActive()) {
            newbrtarget = (br.getCurrentPosition() + (int)(inches * COUNTS_PER_INCH));
            br.setTargetPosition(newbrtarget);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            runtime.reset();
            br.setPower(Math.abs(speed));

            while (opModeIsActive()&& br.isBusy() && runtime.seconds() < timeoutS){
                // Display it for the driver.
                telemetry.addData("Running to",  " %7d ", newbrtarget);
                telemetry.addData("Currently at",  " at %7d", br.getCurrentPosition());
                telemetry.update();
            }
            //code here
            br.setPower(0);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }
    }

