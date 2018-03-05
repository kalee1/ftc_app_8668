package org.firstinspires.ftc.teamcode;
import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/*
 * You can use the X button on either gamepad to turn the LED on and off.
 */
@Disabled
@Autonomous(name = "Sensor Demo", group = "sensor")

public class sensor_demo extends LinearOpMode {
    ColorSensor RGB;
    GyroSensor gyro;
    TouchSensor touch;
    //Servo tester;
    @Override
    public void runOpMode() throws InterruptedException {
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // Assumes config file matches these designations
        //ie: mr, ods, touch, ir
        RGB = hardwareMap.colorSensor.get("mr");
        gyro = hardwareMap.gyroSensor.get("gyro");
        touch = hardwareMap.touchSensor.get("touch");
        // tester = hardwareMap.servo.get("tester");

        //////////////////////////////////
        //    Color Sensor preps       //
        // turn the RGB LED on in the beginning, just so user will know that the sensor is active.
        // hsvValues is an array that will hold the hue, saturation, and value information.
        // bPrevState and bCurrState represent the previous and current state of the button.
        //////////////////////////////////////////////////////////////////////////////////
        RGB.enableLed(true);
        float hsvValues[] = {0F,0F,0F};
//      boolean bPrevState = false;
//      boolean bCurrState = false;
        // wait one cycle.
        waitOneFullHardwareCycle();
        gyro.calibrate();
        while (gyro.isCalibrating())  {
            Thread.sleep(50);           //
        }
        // wait for the start button to be pressed.
        DcMotor rightFront;
        DcMotor leftFront;
        DcMotor rightRear;
        DcMotor leftRear;

        leftFront = hardwareMap.dcMotor.get("leftFront");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightRear = hardwareMap.dcMotor.get("rightRear");
        leftRear.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        RGB.enableLed(true);

        waitForStart();
        while (opModeIsActive()) {

            float yL_val = -gamepad1.left_stick_y;            //reading raw values from the joysticks
            float xL_val = gamepad1.left_stick_x;            //reading raw values from the joysticks
            float xR_val = gamepad1.right_stick_x;
            //clip the right/left values so that the values never exceed +/- 1.
            yL_val = Range.clip(yL_val, -1, 1);
            xL_val = Range.clip(xL_val, -1, 1);
            xR_val = Range.clip(xR_val, -1, 1);
            yL_val = (float) scaleInput(yL_val);
            xL_val = (float) scaleInput(xL_val);
            xR_val = (float) scaleInput(xR_val);
            float RF =(yL_val-xR_val-xL_val);
            float LF =(yL_val+xR_val+xL_val);
            float RR= (yL_val-xR_val+xL_val);
            float LR =(yL_val+xR_val-xL_val);

            RF = Range.clip(RF, -1, 1);
            LF = Range.clip(LF, -1, 1);
            RR = Range.clip(RR, -1, 1);
            LR = Range.clip(LR, -1, 1);

            rightFront.setPower(RF);
            leftFront.setPower(LF);
            rightRear.setPower(RR);
            leftRear.setPower(LR);
            /////////////////////////////////////////////////////////
            //     Color sensor actions                            //
            ////////////////////////////////////////////////////////
            // check the status of the x button on gamepad1.
//            bCurrState = gamepad1.x;
//            // check for button state transitions.
//            if (bCurrState == true && bCurrState != bPrevState)  {
//                // button is transitioning to a pressed state.
//                // update previous state variable.
//                bPrevState = bCurrState;
//                // turn on the LED.
//                RGB.enableLed(true);
//            } else if (bCurrState == false && bCurrState != bPrevState)  {
//                // button is transitioning to a released state.
//                // update previous state variable.
//                bPrevState = bCurrState;
//                // turn off the LED.
//                RGB.enableLed(false);
//            }
            // convert the RGB values to HSV values.
            Color.RGBToHSV(RGB.red()*8, RGB.green()*8, RGB.blue()*8, hsvValues);

            if(RGB.red()>RGB.blue())
            {
                telemetry.addData("","It's Red");
                // tester.setPosition(1);
            }
            if(RGB.blue()>RGB.red())
            {
                telemetry.addData("","It's blue");
                // tester.setPosition(0);
            }
            int xVal = gyro.rawX();
            int yVal = gyro.rawY();
            int zVal = gyro.rawZ();
            int heading = gyro.getHeading();
            if (heading>180)
            {
                heading=360-heading;
                heading=-heading;
            }
            if (heading>0) {

                RF = RF + (heading / 5);
                RR = RR + (heading / 5);
            }
            if (heading<0) {

                LF = LF - (heading / 5);
                LR = LR - (heading / 5);
            }
            RF = Range.clip(RF, -1, 1);
            LF = Range.clip(LF, -1, 1);
            RR = Range.clip(RR, -1, 1);
            LR = Range.clip(LR, -1, 1);

            rightFront.setPower(RF);
            leftFront.setPower(LF);
            rightRear.setPower(RR);
            leftRear.setPower(LR);
            telemetry.addData("01 - White", RGB.alpha());
            telemetry.addData("02 - Red  ", RGB.red());
            telemetry.addData("03 - Green", RGB.green());
            telemetry.addData("04 - Blue ", RGB.blue());
            telemetry.addData("05 - Hue", hsvValues[0]);
            telemetry.addData("14. h", String.format("%03d", heading));
            telemetry.addData("15", touch.isPressed());
            // wait a hardware cycle before iterating.
            waitOneFullHardwareCycle();
        }
    }
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
