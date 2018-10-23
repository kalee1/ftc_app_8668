package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;



import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.STOP_AND_RESET_ENCODER;


@TeleOp(name="Run to Position Test", group="Teleop")

public class motorCheckTeleop extends OpMode
{
    /** The front right drive motor on the robot. */
    protected DcMotor rightFront;
    /** The front left drive motor on the robot. */
    protected DcMotor leftFront;
    /** The right rear drive motor on the robot. */
    protected DcMotor rightRear;
    /** The left rear drive motor on the robot. */
    protected DcMotor leftRear;

    int targetPos = 100;
    double power = 1.0;


    /** Setting the modes and names for all the motors and servos. */
    @Override
    public void init() {
        //Initialize all motors, servos, and sensors, as well as setting some servos to initilaize to a particualr position.
        telemetry.addData ("0", "Beginning init().");

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront.setMode(STOP_AND_RESET_ENCODER);
        rightFront.setMode(STOP_AND_RESET_ENCODER);
        leftFront.setMode(RUN_USING_ENCODER);
        rightFront.setMode(RUN_USING_ENCODER);


        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftRear.setMode(STOP_AND_RESET_ENCODER);
        rightRear.setMode(STOP_AND_RESET_ENCODER);
        leftRear.setMode(RUN_USING_ENCODER);
        rightRear.setMode(RUN_USING_ENCODER);


        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("1", "Ending init().");

    }

    @Override
    public void start()
    {
        leftFront.setMode(RUN_TO_POSITION);
        rightFront.setMode(RUN_TO_POSITION);
        rightRear.setMode(RUN_TO_POSITION);
        leftRear.setMode(RUN_TO_POSITION);

    }

    /** Reading the raw input from the controllers and turning them into movement values for the motors and servos. */
    @Override
    public void loop()
    {

        power = Range.clip(power, -1.0, 1.0);
        targetPos = Range.clip(targetPos, 50, 500);

        if (gamepad1.left_bumper)
        {
            leftFront.setTargetPosition(targetPos);
            leftRear.setTargetPosition(targetPos);
            rightFront.setTargetPosition(targetPos);
            rightRear.setTargetPosition(targetPos);

            leftFront.setPower(power);
            leftRear.setPower(power);
            rightFront.setPower(power);
            rightRear.setPower(power);
        }

        if (gamepad1.dpad_down)
        {
            power -= 0.1;
        }
        else if (gamepad1.dpad_up)
        {
            power += 0.1;
        }

        if (gamepad1.dpad_left)
        {
            targetPos +=25;
        }
        else if (gamepad1.dpad_right)
        {
            targetPos -= 25;
        }

        telemetry.addData("power: ", power);
        telemetry.addData("targetPos: ", targetPos);
        telemetry.addData("LF pos: ", leftFront.getCurrentPosition());
        telemetry.addData("RF pos: ", rightFront.getCurrentPosition());
        telemetry.addData("LR pos: ", leftRear.getCurrentPosition());
        telemetry.addData("RR pos: ", rightRear.getCurrentPosition());



        telemetry.addData("LF is busy: ", leftFront.isBusy());
        telemetry.addData("RF is busy: ", rightRear.isBusy());
        telemetry.addData("LR is busy: ", leftRear.isBusy());
        telemetry.addData("RR is busy: ", rightRear.isBusy());


    }
}
