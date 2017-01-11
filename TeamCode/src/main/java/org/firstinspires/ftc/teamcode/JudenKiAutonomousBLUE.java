package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Juden-Ki mentor, 10 Jan 2017
 * Replaces "JudenKiAutonomous"
*/
@Autonomous(name="Juden-Ki Autonomous Blue", group="Juden-Ki")
public class JudenKiAutonomousBLUE  extends LinearOpMode {
    //Juden Ki Launching Robot
    JudenKiPlatform robot = new JudenKiPlatform();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        DcMotor[] leftMotors = new DcMotor[]{ robot.leftMotorFront, robot.leftMotorBack };
        DcMotor[] rightMotors = new DcMotor[]{ robot.rightMotorFront, robot.rightMotorBack};
        Drive myDrive = new Drive(leftMotors, rightMotors);
        waitForStart();
        // Move forward some
        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }

        //Try to turn clockwise to 45 degrees
        myDrive.gyroTurn2(45, 1);

        myDrive.allStop();

        //Move forward some more
        myDrive.moveForward(24 , 0.8);
        while (myDrive.motorsRunning() == true) {
            myDrive.update();
        }
    }

    public int newHeading (int currentHeading, int turnHeading) {
        int tempHeading;

        tempHeading = currentHeading + turnHeading;

        if (tempHeading > 360)
            tempHeading = tempHeading -360;

        if (tempHeading < 0)
            tempHeading = tempHeading + 360;

        return (tempHeading);
    }
}


*/