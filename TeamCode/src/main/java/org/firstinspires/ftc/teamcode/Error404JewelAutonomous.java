package org.firstinspires.ftc.teamcode;

 /**
 *Error404JewelAutonomous extends <code>Error404_Hardware_Tier2</code> and contains the state
  * machine that steps through each step in the autonomous mission.
  *
  * This year's field presents a unique challenge for autonomous. The field is set up so that there
  * are four unique starting positions and your robot has to be able to start from any of them. We
  * could have four individual programs (one for each quadrant) but this is clumsy and opens us up
  * to bugs. Instead what we did is we made one main autonomous class that holds the framework code
  * that is the same for all four quadrants and then we have four quadrant-specific child classes
  * that supply the quadrant-specific movement vaules.
 *
 * @author Team 8668
 * @see Error404_Hardware_Tier2
 */
public class Error404JewelAutonomous extends Error404_Hardware_Tier2

{
    //////////////////////////////////////////////////////////////////////////////////
    /** This int variable sets the starting case value for the state machine (which step in the sequence to start with -- i.e. the first one or case 0) */
    private int state = 0;
    private int encoder=0;
    private double timer=0;
    /** Initializing the variable that is the distanve from the balancing stone to the cryptobox. */
    protected int cryptoboxDriveDistance;
    protected int glyphPileDriveDistance;
    protected int driveIntoPile;
    /** Initializing the method used for turning the correct side of the robot to face the cryptobox. */
    protected int turnToCryptobox;
    protected int turnToPile;
    /** Initializing the method used for sliding to align on a particular column of the cryptobox. */
    protected int cryptoboxSlide;
    /** A string that indicates which side of the field the robot is on ("blue" or "red") */
    private String fieldSide;
    /** A string that indicates side location on the field ("front" or "back"). */
    private String sideLocation;

    public Error404JewelAutonomous()
    {
    }

    //////////////////////////////////////////////////////////////////////////////////
   /** Gets the gyro heading, sets the jewel servos to the home poition, and grabs the encoder
    * value of the leftFront drive motor for navigation purposes. */
    @Override public void init(){
        super.init();
        telemetry.addData("Gyro: ", getHeading());
        telemetry.addData("","V 1");
        arm.setPosition(0.1);
        swivel.setPosition(0.52);
        encoder=leftFront.getCurrentPosition();

        encoder=leftFront.getCurrentPosition();
    }


    //////////////////////////////////////////////////////////////////////////////////
    /** Determins the field position that robot is starting in and utilizes the coorisponding
     * child class to get the move values for that quadrant of the field. */
    @Override public void start(){
        super.start();
    }

    public void setLocation(String frontOrBack, String redOrBlue){
        fieldSide = redOrBlue;
        sideLocation = frontOrBack;
    }

    protected boolean updateFromVuforia(String cryptoboxKey)
    {
        return true;
    }


    //////////////////////////////////////////////////////////////////////////////////
    /** Contains the state machine which contains the structure for each move the robot will take
     * during autonomous. In the cases where the robot will drive somewhere, there are no values
     * hard-coded in as these values will change depending on what quadrant of the field the robot
     * is starting in. Instead the Autonomous class sources the quadrant-specific movement values
     * from field position-dependant child classes. */
    @Override public void loop ()
    {
        switch (state)
        {
            case 0: // Lower Arm
                arm.setPosition(0.79);
                    state++;
                break;

            case 1:  //Find Jewel
                timer =getRuntime();
                if(camera.getVoltage()<2.1){
                    telemetry.addData("On left","");
                    if(fieldSide.equals("BLUE")){
                        state=3;
                    }
                    if(fieldSide.equals("RED")){
                        state=2;
                    }
                    break;
                }
                else if(camera.getVoltage()>2.1){
                    telemetry.addData("On right","");
                    if(fieldSide.equals("BLUE")){
                        state=2;
                    }
                    if(fieldSide.equals("RED")){
                        state=3;
                    }
                    break;
                }

            case 2:  //Swivel Right
                if(((int)(getRuntime()-timer))>0.3) {
                    swivel.setPosition(0.7);
                    state=4;
                    timer =getRuntime();
                }
                break;

            case 3:  //Swivel Left
                if(((int)(getRuntime()-timer))>0.3) {
                    swivel.setPosition(0.4);
                    state=4;
                    timer =getRuntime();
                }
                break;

            case 4:  //Reset Arm
                if(((int)(getRuntime()-timer))>0.1) {
                    arm.setPosition(0.1);
                    swivel.setPosition(0.5);
                    state=6;
                    timer =getRuntime();
                }
                break;

            case 6:  //Read Pictograph
                if(updateFromVuforia(readCryptograph()))
                {
                    state= 7;
                }


                if(((int)(getRuntime()-timer))>2)
                {
                    updateFromVuforia("CENTER");
                    state=7;
                }
                break;


            case 7:  //Drive to Pile
                if(glyphPileDriveDistance!=0)
                {
                    if(glyphPileDriveDistance>0)
                    {
                        driveStraightCombo(0.7);
                    }
                    else
                    {
                        driveStraightCombo(-0.7);
                    }

                    if (leftFront.getCurrentPosition() - encoder > Math.abs(glyphPileDriveDistance))
                    {
                        stopEverything();
                        setMultipleDirections("turn", "right");
                        state++;
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("turn", "right");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }

                break;

            case 8:  //Face Glyph Pile
                if(turnToPile!=0)
                {
                    pointTurnCombo(0.6);
                    if (Math.abs(getHeading()) > turnToPile)
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("slide", "left");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }
                break;

            case 9:  // Drive Into GLyph Pile
                if(driveIntoPile!=0)
                {
                    if(driveIntoPile>0)
                    {
                        driveStraightCombo(0.7);
                    }
                    else
                    {
                        driveStraightCombo(-0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(driveIntoPile))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 10:  //Collect Glyphs From Pile
                glyphIntake("in");
                driveStraightCombo(0.7);
                if(leftFront.getCurrentPosition()-encoder>50)
                {
                    stopEverything();
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 11:  //Orient on Cryptobox
                if(turnToCryptobox!=0)
                {
                    pointTurnCombo(0.6);
                    if (Math.abs(getHeading()) > turnToCryptobox)
                    {
                        state++;
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        encoder = leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("slide", "left");
                    state++;
                    encoder = leftFront.getCurrentPosition();
                }
                break;

            case 12:  // Drive To Cryptobox
                if(cryptoboxDriveDistance!=0)
                {
                    if(cryptoboxDriveDistance>0)
                    {
                        driveStraightCombo(0.7);
                    }
                    else
                    {
                        driveStraightCombo(-0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(cryptoboxDriveDistance))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 13:  // Slide to Cryptobox
                if(cryptoboxSlide!=0)
                {
                    if(cryptoboxSlide>0)
                    {
                        slideSidewaysCombo(0.7);
                    }
                    else
                    {
                        slideSidewaysCombo(-0.7);
                    }

                    if(leftFront.getCurrentPosition()-encoder>Math.abs(cryptoboxSlide))
                    {
                        stopEverything();
                        setMultipleDirections("straight", "forward");
                        state++;
                        encoder=leftFront.getCurrentPosition();
                    }
                }
                else
                {
                    setMultipleDirections("straight", "forward");
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 14:  //Drive into Cryptobox
                driveStraight("RUE",0.6,"f",0);
                if(leftFront.getCurrentPosition()-encoder>150)
                {
                    driveStraight("RUE",0,"r",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 15:  //Deploy Glyph
                glyphIntake("out");
                state++;
                break;

            case 16:  //Wait
                if(leftGlyph.getCurrentPosition()>20)
                {
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;

            case 17:  // Back Up
                driveStraight("RUE",0.7,"r",0);
                if(leftFront.getCurrentPosition()-encoder>130)
                {
                    driveStraight("RUE",0,"f",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                    glyphIntake("stop");
                }
                break;
            case 18: // Push Glyph in
                glyphIntake("outSlow");
                driveStraight("RUE",0.3,"f",0);
                if(leftFront.getCurrentPosition()-encoder>150)
                {
                    driveStraight("RUE",0,"r",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;
            case 19:
                driveStraight("RUE",0.7,"r",0);
                if(leftFront.getCurrentPosition()-encoder>100)
                {
                    driveStraight("RUE",0,"f",0);
                    state++;
                    encoder=leftFront.getCurrentPosition();
                }
                break;
            default:
                glyphIntake("stop");
                break;


        }
        telemetry.addData("1. State: ", state);
        telemetry.addData("2. Gyro: ", getHeading());
        telemetry.addData("3. Camera:  ", camera.getVoltage());
        telemetry.addData("4. Left Front Position: ", leftFront.getCurrentPosition());
        telemetry.addData("5. Delta Position: ", encoder);
        telemetry.addData("6. Cryptobox Drive Distance: ", cryptoboxDriveDistance);
        telemetry.addData("7. Cryptobox Slide: ", cryptoboxSlide);
        telemetry.addData("Pattern: ", readCryptograph());




    } // loop
} //
