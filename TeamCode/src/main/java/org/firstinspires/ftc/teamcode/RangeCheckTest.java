package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="rangeTest", group="Jewel")


public class RangeCheckTest extends Error404_Hardware_Tier2
{
    protected RangeCheck distanceCheckBottom = new RangeCheck(360, 52, 3);

    @Override public void loop()
    {
        boolean result = distanceCheckBottom.checkRange( bottomRange.cmUltrasonic() );

        telemetry.addData("6. Bottom wall Distance: ", bottomRange.cmUltrasonic() );
        telemetry.addData("rangeCheck result: ", result);
        telemetry.addData("length: ", distanceCheckBottom.getLength());



    }

}
