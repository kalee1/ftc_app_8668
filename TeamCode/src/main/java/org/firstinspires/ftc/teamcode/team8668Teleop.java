package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.kauailabs.NavxMicroNavigationSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODERS;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODERS;

@TeleOp(name="8668 Teleop meccanum", group="Teleop")

/**
 * team8668Teleop extends the <code>OpMode</code> class.
 *
 * Contains all of the button and joy-stick initializations for moving the robot with the
 * remotes as well as the motor and servo initializations for Teleop.
 *
 * @author Team 8668
 * @see OpMode
 */
public class team8668Teleop extends OpMode {

    /** The front right drive motor on the robot. */
    protected DcMotor rightFront;
    /** The front left drive motor on the robot. */
    protected DcMotor leftFront;
    /** The right rear drive motor on the robot. */
    protected DcMotor rightRear;
    /** The left rear drive motor on the robot. */
    protected DcMotor leftRear;
    /** The left glyph intake motor -- used in conjunction with the rightGlyph motor to suck in glyphs. */
    protected DcMotor leftGlyph;
    /** The right glyph intake motor -- used in conjunction with the leftGlyph motor to suck in glyphs. */
    protected DcMotor rightGlyph;

    /** The arm servo raises and lowers the jewel arm. */
    protected Servo arm;

    /** The swivel servo swings the jewel sword back and forth to knock off the glyph. */
    protected Servo swivel;
    /** The shoulder servo is a high-torque servo that extends the relic arm up and over the field wall. */
    protected Servo shoulder;
    /** The elbow servo raises and lowers the relic for extra flexibility when deploying the relic. */
    protected Servo elbow;
    /** The hand servo controls the claw that grabs the relic. */
    protected Servo hand;
    /** Spins the tracks on the glyph tray, spitting the glyph off -- not used. */
    protected Servo glyphTrayTilt;

    protected IntegratingGyroscope gyro;
    /** The navxMicro is a gyro and is used to record the robot's heading. */
    protected NavxMicroNavigationSensor navxMicro;

    /** Moving average window used to filter the leftStick_y value. */
    protected MovingAverage leftStick_y = new MovingAverage(1);
    /** Moving average window used to filter the leftStick_x value. */
    protected MovingAverage leftStick_x = new MovingAverage(1);
    /** Moving average window used to filter the rightStick_x value. */
    protected  MovingAverage rightStick_x = new MovingAverage(1);

    /** Setting the start position of the elbow servo. */
    protected double elbowPos=1;
    /** Setting the start position of the shoulder servo. */
    protected double shoulderPos=0.95;
    /** Setting the starting position for the swivel servo. */
    protected double swivelPos =0.522;
    /** Setting the start position for the hand servo. */
    protected double handPos=0.7;
    /** Setting the start position for the glyphTrayTilt servo. */
    protected double tiltPosition=0.05;


    int gyroX=0;
    int gyroY=0;
    int xdelta=0;
    int ydelta=0;


    /** Setting the modes and names for all the motors and servos. */
    @Override
    public void init() {
        //Initialize all motors, servos, and sensors, as well as setting some servos to initilaize to a particualr position.
        telemetry.addData ("0", "Beginning init().");

        arm=hardwareMap.servo.get("jewelSword");

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");

        leftFront.setMode(RUN_USING_ENCODER);
        rightFront.setMode(RUN_USING_ENCODER);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);

        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        rightRear.setMode(RUN_USING_ENCODER);
        leftRear.setMode(RUN_USING_ENCODER);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        shoulder = hardwareMap.servo.get("shoulder");

        glyphTrayTilt = hardwareMap.servo.get("glyphTilt");

        elbow = hardwareMap.servo.get("elbow");

        hand = hardwareMap.servo.get("hand");

        swivel=hardwareMap.servo.get("jewelSwivel");

        leftGlyph = hardwareMap.dcMotor.get("leftGlyph");

        rightGlyph = hardwareMap.dcMotor.get("rightGlyph");

        leftGlyph.setMode(RUN_USING_ENCODER);
        rightGlyph.setMode(RUN_USING_ENCODER);
        leftGlyph.setDirection(DcMotor.Direction.REVERSE);
        rightGlyph.setDirection(DcMotor.Direction.REVERSE);

        hand.setPosition(0.7);
        arm.setPosition(0.1);
        swivel.setPosition(0.5);

        try {
            navxMicro = hardwareMap.get(NavxMicroNavigationSensor.class, "navx");
            gyro = (IntegratingGyroscope)navxMicro;
        } catch (Exception p_exeception) {
            telemetry.addData("navx not found in config file", 0);
            navxMicro = null;
        }

        telemetry.addData("1", "Ending init().");

    }
    /**  The start method. The gyro starts as soon as the class is selected on the driver controller. */
    @Override
    public void start()
    {
        Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        xdelta=(int)AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));

        Orientation angles2 = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        ydelta=(int)AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles2.angleUnit, angles2.secondAngle));
    }

    /** Reading the raw input from the controllers and turning them into movement values for the motors and servos. */
    @Override
    public void loop() {

        //////////////////////////////////////////
        /////Drive Train//////////////////////////
        /////////////////////////////////////////

        leftStick_y.add(-gamepad1.left_stick_y);    //assigning joystick and axis to filter window
        leftStick_x.add(gamepad1.left_stick_x);    //assigning joystick and axis to filter window
        rightStick_x.add(gamepad1.right_stick_x);    //assigning joystick and axis to filter window

        float yL_val = (float) leftStick_y.getValue();    //getting smoothed values from left joystick on the y-axis
        float xL_val = (float) leftStick_x.getValue();    //getting smoothed values from left joystick on the x-axis
        float xR_val = (float) rightStick_x.getValue();    //getting smoothed values from right joytick on the x-axis

        //clipping all incoming values to make sure that they don't exceed +/- 1
        yL_val = Range.clip(yL_val, -1, 1);
        xL_val = Range.clip(xL_val, -1, 1);
        xR_val = Range.clip(xR_val, -1, 1);

        float RF = (yL_val - xR_val - xL_val);  //these are the calculations need to make a simple
        float LF = (yL_val + xR_val + xL_val);  // mecaccnum drive. The left joystick controls moving
        float RR = (yL_val - xR_val + xL_val);  //straight forward/backward and straight sideways. The
        float LR = (yL_val + xR_val - xL_val);  //right joystick controls turning.

        //Find the largest command value given and assign it to max.
        double max = Math.abs(LF);
        if (Math.abs(RF) > max) {
            max = Math.abs(RF);
        }
        if (Math.abs(LR) > max) {
            max = Math.abs(LR);
        }
        if (Math.abs(RR) > max) {
            max = Math.abs(RR);
        }

        // If max is greater than 1, divide all command values by max to ensure that all command
        // values stay below a magnitude of 1.
        if (max > 1) {
            LF /= max;
            RF /= max;
            LR /= max;
            RR /= max;
        }


//        if (gamepad1.y) {
//            arm.setPosition(0.79);    //move down jewel arm
//        } else if (gamepad1.a) {
//            arm.setPosition(0.1);     //move up jewel arm
//        }

        //////////////////////////////////////////
        ///// Relic Arm //////////////////////////
        //////////////////////////////////////////

        // Shoulder of Relic Arm
        if (gamepad1.dpad_down) {
            shoulderPos += 0.005;       //increment shoulder servo
        } else if (gamepad1.dpad_up) {
            shoulderPos -= 0.005;       //increment shoulder servo down
        }
        if (gamepad1.dpad_left) {
            shoulderPos = 0.7;
        } else if (gamepad1.dpad_right) {
            shoulderPos = 0.95;
        }
        shoulderPos = Range.clip(shoulderPos, 0.7, 0.95); //keep shoulder servo value in given range


        // Elbow of Relic Arm
        if (gamepad1.y) {
            elbowPos += 0.006;
            //increment elbow out
        } else if (gamepad1.a) { //increment elbow in
            elbowPos -= 0.006;
        }
        elbowPos = Range.clip(elbowPos, 0, 1); //keep elbow servo value in given range


        // Claw of Relic Arm
//        if (gamepad2.right_bumper) {
//            handPos = 0.4;                      //open grabber
//        }
//        if (gamepad2.left_bumper) {
//            handPos = 0.65;                      //close grabber
//        }
        if (gamepad1.x) {      //increment grabber open
            handPos -= 0.01;
        }

        if (gamepad1.b ) {       //increment grabber close
            handPos += 0.01;
        }

        //////////////////////////////////////////
        ///// Jewel Sword ////////////////////////
        //////////////////////////////////////////

        // Jewewl Servo
//        if (gamepad1.x) {                       //turn jewel servo to the side
//            swivelPos = 0.40;
//        } else if (gamepad1.b) {                      //turn jewel servo to the other side
//            swivelPos = 0.64;
//        } else {
//            swivelPos = 0.52;
//        }

//
//        if(gamepad1.right_stick_button){
//
//            Orientation angles = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gyroX=(int)AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles.angleUnit, angles.thirdAngle));
//
//            Orientation angles2 = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//            gyroY=(int)AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angles2.angleUnit, angles2.secondAngle));
//
//            RF =((((gyroY-ydelta)/(float)(1.5))+(gyroX-xdelta))/7);  //these are the calculations need to make a simple
//            LF =(((gyroY-ydelta)/(float)(1.5))-(gyroX-xdelta))/7;  // mecaccnum drive. The left joystick controls moving
//            RR= (((gyroY-ydelta)/(float)(1.5))-(gyroX-xdelta))/7;  //straight forward/backward and straight sideways. The
//            LR =(((gyroY-ydelta)/(float)(1.5))+(gyroX-xdelta))/7;  //right joystick controls turning.
//
//            RF = Range.clip(RF, -1, 1);          //make sure power stays between -1 and 1
//            LF = Range.clip(LF, -1, 1);
//            RR = Range.clip(RR, -1, 1);
//            LR = Range.clip(LR, -1, 1);
//        }

        Range.clip(handPos, 0.35, 0.8);

        rightFront.setPower(RF);              //Set all values to correct devices
        leftFront.setPower(LF);
        rightRear.setPower(RR);
        leftRear.setPower(LR);

        elbow.setPosition(elbowPos);
        shoulder.setPosition(shoulderPos);
        swivel.setPosition(swivelPos);
        hand.setPosition(handPos);

        double intake_power = 0;
        if (gamepad1.left_trigger > 0)
        {
            intake_power = 1.0;
        }
        else if (gamepad1.left_bumper){
            intake_power = -1.0;
        }

//        rightGlyph.setPower(gamepad1.right_trigger-gamepad1.right_bumper);
//        leftGlyph.setPower(gamepad1.right_trigger-gamepad1.right_bumper);

        rightGlyph.setPower(intake_power);
        leftGlyph.setPower(intake_power);



        //Tilting the glyph tray towards the cryptobox
        tiltPosition+=(gamepad2.right_stick_y)*(0.003);
        tiltPosition=Range.clip(tiltPosition, 0.05, 0.267);  //Min and max for glyphTilt Servo
        glyphTrayTilt.setPosition(tiltPosition);


      telemetry.addData("shoulder: ",shoulder.getPosition());
      telemetry.addData("grabber:",hand.getPosition());          //print info to telemetry
      telemetry.addData("elbow: ",elbow.getPosition());
      telemetry.addData("swivel: ",swivel.getPosition());
      telemetry.addData("hand: ",handPos);
      telemetry.addData("gyro X: ",gyroX);
      telemetry.addData("gyro Y: ",gyroY);
      telemetry.addData("glyph Tray Tilt: ",glyphTrayTilt.getPosition());
      telemetry.addData("Right Rear Position: ", rightRear.getCurrentPosition());
      telemetry.addData("Right Front Position: ", rightFront.getCurrentPosition());
      //telemetry.addData("glyphter position: ", encoderMotor.getCurrentPosition());
      //telemetry.addData("glyphter position with delta: ", (encoderMotor.getCurrentPosition()-encoderDelta));
      //telemetry.addData("glyphter speed: ", glyphterSpeed);
      //telemetry.addData("glyphter bottom: ", !bottom.getState());
      //telemetry.addData("glyphter top: ", !top.getState());


    }

    /** Stop everything */
    @Override
    public void stop() {
    }

    /**
     * Takes the raw data input from the joystick and returns it as a curve to help with smoother robot movement.
     *
     * @param dVal The value of the d stick recorded as a double. */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00};
        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
