package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Red Front", group="Jewel")

public class e404_FrontRed extends Error404JewelAutonomous

{
    ///////////////////////////////////////////////////////////////////

    public e404_FrontRed()
    {
        setLocation("FRONT", "RED");
    }
    @Override public void init()
    {
        cryptoboxDriveDistance = 780;
        cryptoboxSlide=0;
        turnToCryptobox=80;
        setMultipleDirections("straight", "forward");
        super.init();  //super.init() method is moved to bottom to not get in the way of the driveStraight() method
    }
    @Override public void start(){
        super.start();
    }

    @Override protected boolean updateFromVuforia(String cryptoboxKey)
    {
        boolean result = false;

        if(cryptoboxKey.equals("LEFT"))
        {
            cryptoboxDriveDistance = 920;
            result = true;
        }
        else if(cryptoboxKey.equals("RIGHT"))
        {
            cryptoboxDriveDistance=620;
            result = true;
        }
        else if(cryptoboxKey.equals("CENTER"))
        {
            cryptoboxDriveDistance=780;
            result = true;
        }
        return result;
    }

    @Override public void loop ()
    {
        super.loop();
    } // loop

}
