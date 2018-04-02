package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.robotcontroller.external.samples.SensorMRRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * {@link RangeSensorTest} illustrates how to use the Modern Robotics
 * Range Sensor.
 *
 * The op mode assumes that the range sensor is configured with a name of "sensor_range".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 *
 * @see <a href="http://modernroboticsinc.com/range-sensor">MR Range Sensor</a>
 */
@Autonomous(name = "Sensor: MR range sensor", group = "Sensor")
//@Disabled   // comment out or remove this line to enable this opmode

public class RangeSensorTest extends OpMode
{
    ModernRoboticsI2cRangeSensor upperRangeSensor, lowerRangeSensor;

    public void init() {
        // get a reference to our compass
        upperRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "topRange");
        lowerRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bottomRange");

    }

    public void loop()
    {
        telemetry.addData("------- Upper Ultrasonic ------", "");

        telemetry.addData("raw ultrasonic", upperRangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", upperRangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", upperRangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", upperRangeSensor.getDistance(DistanceUnit.CM));

        telemetry.addData("------- Lower Ultrasonic ------", "");
        telemetry.addData("raw ultrasonic", lowerRangeSensor.rawUltrasonic());
        telemetry.addData("raw optical", lowerRangeSensor.rawOptical());
        telemetry.addData("cm optical", "%.2f cm", lowerRangeSensor.cmOptical());
        telemetry.addData("cm", "%.2f cm", lowerRangeSensor.getDistance(DistanceUnit.CM));


    }

}
