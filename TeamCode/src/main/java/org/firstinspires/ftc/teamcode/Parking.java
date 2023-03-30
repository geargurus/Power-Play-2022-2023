package org.firstinspires.ftc.teamcode.Encoders;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Parking", group="Robot")

public class Parking extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor         fr   = null;
    private DcMotor         fl  = null;
    private DcMotor         bl  = null;
    private DcMotor         br  = null;
    private DcMotor VL = null;
    private DcMotor VR = null;
    private CRServo intake = null;

    private ElapsedTime     runtime = new ElapsedTime();

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    //Neco-Arc waz hear
    static final double     COUNTS_PER_MOTOR_REV    = 312 ;
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_INCHES   = 3.7 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;

    static final double     COUNTS_PER_SPOOL_MOTOR_REV    = 2786.2  ;
    static final double     DRIVE_SPOOL_GEAR_REDUCTION    =  1 ;     // This is < 1.0 if geared UP
    static final double     SPOOL_DIAMETER_INCHES   = 2.2 ;     // For figuring circumference
    static final double     ROTATION_PER_INCH         = (COUNTS_PER_SPOOL_MOTOR_REV * DRIVE_SPOOL_GEAR_REDUCTION) /
            (SPOOL_DIAMETER_INCHES * 3.1415);

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        fr  = hardwareMap.get(DcMotor.class, "frontRight");
        fl = hardwareMap.get(DcMotor.class, "frontLeft");
        br  = hardwareMap.get(DcMotor.class, "backRight");
        bl  = hardwareMap.get(DcMotor.class, "backLeft");
        VL = hardwareMap.get(DcMotor.class,  "ViperLeft");
        VR = hardwareMap.get(DcMotor.class, "ViperRight");
        intake = hardwareMap.get(CRServo.class,  "inTake" );

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        fr.setDirection(DcMotor.Direction.REVERSE);
        fl.setDirection(DcMotor.Direction.FORWARD);
        br.setDirection(DcMotor.Direction.REVERSE);
        bl.setDirection(DcMotor.Direction.FORWARD);

        fr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        br.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        VR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        VR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Starting at",  "%7d :%7d :%7d :%7d :%7d :7d%",
                fr.getCurrentPosition(),
                fl.getCurrentPosition(),
                br.getCurrentPosition(),
                bl.getCurrentPosition(),
                VL.getCurrentPosition(),
                VR.getCurrentPosition());
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        encoderDrive(DRIVE_SPEED,  -30,  -30, 5.0);


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(3000);
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderCascade(double speed,
                               double Inches,
                               double timeoutS) {
        int newCascadeTarget;


        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller

            newCascadeTarget = VL.getCurrentPosition() + (int)(Inches * ROTATION_PER_INCH);
            VL.setTargetPosition(newCascadeTarget);

            newCascadeTarget = VR.getCurrentPosition() + (int)(Inches * ROTATION_PER_INCH);
            VR.setTargetPosition(newCascadeTarget);

            // Turn On RUN_TO_POSITION

            VL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            VR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();

            VL.setPower(Math.abs(speed));
            VR.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (VL.isBusy() && VR.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d",  newCascadeTarget);
                telemetry.addData("Currently at",  " at %7d", newCascadeTarget,
                        VL.getCurrentPosition(), VR.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;

            VL.setPower(0);
            VR.setPower(0);

            // Turn off RUN_TO_POSITION

            VL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            VR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newflTarget;
        int newfrTarget;
        int newblTarget;
        int newbrTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newflTarget = fl.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newfrTarget = fr.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            newblTarget = bl.getCurrentPosition() + (int)(leftInches * COUNTS_PER_INCH);
            newbrTarget = br.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            fl.setTargetPosition(newflTarget);
            fr.setTargetPosition(newfrTarget);
            bl.setTargetPosition(newblTarget);
            br.setTargetPosition(newbrTarget);

            // Turn On RUN_TO_POSITION
            fr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            fl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            br.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            bl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            fr.setPower(Math.abs(speed));
            fl.setPower(Math.abs(speed));
            br.setPower(Math.abs(speed));
            bl.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (fr.isBusy() && br.isBusy() && bl.isBusy() && fl.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Running to",  " %7d :%7d :%7d :%7d", newfrTarget,  newflTarget, newbrTarget, newblTarget);
                telemetry.addData("Currently at",  " at %7d :%7d :%7d :%7d", newfrTarget, newflTarget, newbrTarget, newblTarget,
                        fr.getCurrentPosition(), fl.getCurrentPosition(), br.getCurrentPosition(), bl.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            fr.setPower(0);
            fl.setPower(0);
            br.setPower(0);
            bl.setPower(0);

            // Turn off RUN_TO_POSITION
            fr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            fl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            br.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }
}