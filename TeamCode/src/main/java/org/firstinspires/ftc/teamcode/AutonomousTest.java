package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Func;

public class AutonomousTest extends OpMode
{
    Chassis theChassis;
    Vision theVision;
    GlyphIntake theIntake;

    /** This int variable sets the starting case value for the state machine (which step in the sequence to start with -- i.e. the first one or case 0) */
    private int state = 0;
    private double timer=0;
    /** A move variable for driving from the glyph pile to the cryptobox. */
    protected int cryptoboxDriveDistance;
    /** A move variable for driving to line up on the glyph pile. */
    protected int glyphPileSlideDistance;
    /** A move variable for driving the robot into the glyph pile. */
    protected int driveIntoPile;
    /** Initializing the turning variable used for turning the correct side of the robot to face the cryptobox. */
    protected int turnToCryptobox;
    /** A move variable for turing the robot to face the glyph pile. */
    protected int turnToPile;
    /** Initializing the variable used for sliding the robot side to side to align on a particular column of the cryptobox. */
    protected int cryptoboxSlide;
    protected double distanceFromWall;
    protected int slideToEdge;

    /** A string that indicates which side of the field the robot is on ("blue" or "red") */
    private String fieldSide;

    private String theColumn = "NONE";
    public boolean complete = false;

    /** Determines the field position that robot is starting in and utilizes the coorisponding
     * child class to get the move values for that quadrant of the field. */
    public void setLocation(String redOrBlue)
    {
        fieldSide = redOrBlue;
    }

    @Override
    public void init()
    {
        theChassis.init( hardwareMap, telemetry );
        theVision.init( hardwareMap, telemetry );
        theIntake.init( hardwareMap, telemetry );
    }

    @Override
    public void start()
    {

    }

    @Override
    public void loop()
    {
        switch( state )
        {
            case 0: // Lower Arm

 //               arm.setPosition(0.79);
                state++;
                break;

            case 1:  //Find Jewel

                timer = getRuntime();
                String jewelLocation = theVision.redJewelLocation();

                if( jewelLocation.equals("LEFT") )
                {
                    telemetry.addData("On left","");
                    if(fieldSide.equals("BLUE")){
                        state=3;
                    }
                    if(fieldSide.equals("RED")){
                        state=2;
                    }
                }
                else if( jewelLocation.equals("RIGHT") )
                {
                    telemetry.addData("On right","");
                    if(fieldSide.equals("BLUE")){
                        state=2;
                    }
                    if(fieldSide.equals("RED")){
                        state=3;
                    }
                }
                break;

            case 2:  //Swivel Right

                if( (getRuntime()-timer) > 0.3 )
                {
//                    swivel.setPosition(0.7);
                    state=4;
                    timer = getRuntime();
                }
                break;

            case 3:  //Swivel Left

                if( (getRuntime()-timer) > 0.3 )
                {
//                    swivel.setPosition(0.4);
                    state=4;
                    timer = getRuntime();
                }
                break;

            case 4:  //Reset Arm

                if((getRuntime()-timer) > 0.1 )
                {
//                    arm.setPosition(0.1);
//                    swivel.setPosition(0.5);
                    state++;
                    timer = getRuntime();
                }
                break;

            case 5:  //Read Pictograph

                theColumn = theVision.readCryptograph();

                if( (getRuntime()-timer) > 2.0 )
                {
                    updateFromVuforia("CENTER");
                    state++;
                }
                else if ( updateFromVuforia( theColumn ) )
                {
                    state++;
                }
                break;

            case 6:  // Slide to Glyph Pile

                double thePower = 0.7;
                if(glyphPileSlideDistance < 0.0 )
                {
                    thePower = -1.0 * thePower;
                }

                complete = theChassis.slideSidewaysDistance( thePower, glyphPileSlideDistance );

                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 7:  //Face Glyph Pile

                complete = theChassis.pointTurnGyro(turnToPile, 3.0, Math.abs(turnToPile) > 90.0 );

                if ( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 8:  // Drive Into Glyph Pile

                theIntake.in();

                thePower = 0.7;
                if(driveIntoPile < 0)
                {
                    thePower = -1.0*thePower;
                }

                complete = theChassis.driveStraightTimed( thePower, 1.25 );

                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 9:  //Collect Glyphs From Pile

                complete = theChassis.driveStraightDistance(0.3, 100);
                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 10:  // Back up from glyph pile

                complete = theChassis.driveStraightDistance(-0.7, 100);
                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 11:  //Orient on Cryptobox

                complete = theChassis.pointTurnGyro( turnToCryptobox, 2.5, Math.abs(turnToCryptobox) > 90.0 );
                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 12:  // Drive To Cryptobox

                complete = theChassis.driveStraightDistanceGyro(-0.5, cryptoboxDriveDistance, turnToCryptobox );

                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 15:  // Line up on a cryptobox column

                thePower = 0.7;
                if(cryptoboxSlide < 0.0 )
                {
                    thePower = -1.0*thePower;
                }
                complete = theChassis.slideSidewaysDistance( thePower, cryptoboxSlide );

                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;

            case 16:  //Drive into Cryptobox

                complete = theChassis.driveStraightDistance( -0.6, 25);
                if( complete )
                {
                    state++;
                    complete = false;
                    timer = getRuntime();
                }
                break;

            case 17:  //Deploy Glyph
//                glyphter.setPosition(0.23);
                state++;
                complete = false;
                break;

            case 18:  //Wait
                if(getRuntime()-timer>2.5)
                {
//                    glyphter.setPosition(.05);
                    state++;
                    complete = false;
                }
                break;

            case 19:  // Back Up
                complete = theChassis.driveStraightDistance(0.7, 60);
                if( complete )
                {
                    state++;
                    complete = false;
                    theIntake.stop();
                }
                break;

            case 20: // Push Glyph in
                complete = theChassis.driveStraightDistance(-0.3, 40 );
                if( complete )
                {
                    state++;
                    complete = false;
                }
                break;
            case 21:  //drive away from cryptobox into safe zone
                complete = theChassis.driveStraightDistance(0.5, 50 );
                if( complete )
                {
                    state++;
                    theChassis.stopEverything();
                }
                break;

            default:
                break;

        }
        telemetry.addData("1. State: ", state);
        telemetry.addData("2. Gyro: ", "%.3f degrees", new Func<Double>() { @Override public Double value() { return theChassis.getHeadingDbl(); } } );
        telemetry.addData("3. Camera:  ", "0.3f volts", new Func<Double>() { @Override public Double value() { return theVision.camera.getVoltage(); } } );
        telemetry.addData("4. Left Front Position: ", new Func<Integer>() { @Override public Integer value() { return theChassis.getCurrentPosition(); } } );
        telemetry.addData("5. Delta Position: ", theChassis.initialEncoder );
        telemetry.addData("6. Pattern: ", theColumn );

    }

    protected boolean updateFromVuforia(String cryptoboxKey)
    {
        return true;
    }
}
