package org.firstinspires.ftc.teamcode;

public class RearBlue extends AutonomousTest
{

    /** Setting the movement distances for this quadrant. */
    @Override
    public void init()
    {
        setLocation("BLUE");

        //Drive distance variables
        glyphPileSlideDistance = 375;
        driveIntoPile = 250;
        cryptoboxDriveDistance = 500;
        cryptoboxSlide=270;

        //Turn angel and range variables
        turnToPile = 45;
        turnToCryptobox=0;
        distanceFromWall = 65.70;
        slideToEdge = 37;
        super.init();  //super.init() method is moved to bottom to not get in the way of the chassis_driveStraight() method
    }

    @Override
    protected boolean updateFromVuforia(String cryptoboxKey)
    {
        boolean result = false;

        if(cryptoboxKey.equals("LEFT"))
        {
            cryptoboxSlide = 360;
            result = true;
        }
        else if(cryptoboxKey.equals("RIGHT"))
        {
            cryptoboxSlide=180;
            result = true;
        }
        else if(cryptoboxKey.equals("CENTER"))
        {
            cryptoboxSlide=270;
            result = true;
        }
        return result;
    }
}
