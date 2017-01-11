package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by judenki on 11/26/16.
 *  revised by Juden-Ki mentors on 10 Jan 17.
 *
 * Actuators and Sensors for Driver Control Mode
 */

//@TeleOp(name="juden Ki Driver Mode", group="Juden Ki")
public class JudenKiDriverMode extends LinearOpMode {

    public JudenKiPlatform robot = new JudenKiPlatform();
    //public HardwareMap hardwareMap = null; // will be set in OpModeManager.runActiveOpMode


    @Override
    public void runOpMode() throws InterruptedException {
        double forward;
        double drift;
        /*
         * Probably want to move this to platform class as both autonomous and driver use it.
        */
        robot.init(hardwareMap);

        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Drivers");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double prev_YStickPos = 0;
            double prev_XStickPos = 0;

            double overDrive = (gamepad1.right_bumper) ? 1.0 : 0.6;
            forward = stickFilter(gamepad1.left_stick_y * overDrive, prev_YStickPos);
            drift = stickFilter(gamepad1.left_stick_x, prev_XStickPos);
            //forward = gamepad1.left_stick_y;

            myDrive.driveMove(forward, drift);

            //Actuate Catapult
            if (gamepad1.a)
                robot.catapultMotor.setPower(.5);
            if (gamepad1.b)
                robot.catapultMotor.setPower(.0);

            // Send telemetry message to signify robot running;
            telemetry.addData("left x",  "%.2f", gamepad1.left_stick_x);
            telemetry.addData("right x", "%.2f", gamepad1.right_stick_x);
            telemetry.addData("heading", robot.gyro.getHeading());
/*            telemetry.addData("Bottom    Red   ", "%d", robot.colorTheBottom.red());
            telemetry.addData("Bottom    Green ", "%d", robot.colorTheBottom.green());
            telemetry.addData("Bottom    Blue  ", "%d", robot.colorTheBottom.blue());
            telemetry.addData("Side      Red   ", "%d", robot.colorTheSide.red());
            telemetry.addData("Side      Green ", "%d", robot.colorTheSide.green());
            telemetry.addData("Side      Blue  ", "%d", robot.colorTheSide.blue());
*/
            telemetry.addData("Catapult Fried  Dill  Pickle  touch ", robot.touchCat.isPressed()    );
            telemetry.update();



            // Pause for metronome tick.  40 mS each cycle = update 25 times a second.
            waitForTick(40);
            idle(); // Always call idle() at the bottom of your while(opModeIsActive()) loop
        }
    }
    private ElapsedTime period  = new ElapsedTime();
    public void waitForTick(long periodMs)  throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
    /************************************************************************/
    /*                     filter_input                                     */
    /*      Read the inputs, post process them, and produce the             */
    /*      output Motor Drive commands                                     */
    /*   Inputs: stick position, previous stick position                    */
    /*   Outputs: (return value)                                            */
    /*   Returns:  Filtered stick position                                  */
    /************************************************************************/
    public double stickFilter(double inStick, double prevStickPos) {
        double HI_FILT_CONST = 0.96;
        double MID_FILT_CONST = 0.5;
        double LOW_FILT_CONST = 0.1;
        double filterConstant = 0;  /*init to zero just in case */
        /* cube the input stick to give better response around zero */
        instick = instick * instick * instick;
        /* pick a filter constant based on stick position */
        if (Math.abs(inStick) > 0.85) {
            filterConstant = (double) HI_FILT_CONST;
        } else if (Math.abs(inStick) > 0.6) {
            filterConstant = (double) MID_FILT_CONST;
        } else if (Math.abs(inStick) > 0.2) {
            filterConstant = (double) LOW_FILT_CONST;
        }
	/*  Basic formula for a simple 1st order filter is:      */
	/*      ((1-FC)* input) + (FC * prev_value)              */
        prevStickPos = ((1 - filterConstant) * inStick) + (filterConstant * prevStickPos);
        return (prevStickPos);
    }


}