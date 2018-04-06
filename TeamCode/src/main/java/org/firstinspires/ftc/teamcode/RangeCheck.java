package org.firstinspires.ftc.teamcode;

/**
 * Created by andrew on 4/5/18.
 */

public class RangeCheck {

    private double upperLimit;
    private double lowerLimit;
    private int length = 0;
    private int lengthLimit;

    RangeCheck(double upper, double lower, int limit)
    {
        upperLimit = upper;
        lowerLimit = lower;
        lengthLimit = limit;
    }

    public boolean checkRange (double rangeNum)
    {
        if ( rangeNum < upperLimit && rangeNum > lowerLimit)
        {
            length = 0;
        }
        else
        {
            length++;
        }

        if(length >= lengthLimit)
        {

            return true;
        }
        else
        {
            return false;
        }

    }

    public int getLength()
    {
        return length;
    }



}
