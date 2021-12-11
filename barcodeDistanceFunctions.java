public class barcodeDistanceFunctions
{
	//test if element is in directily in front
	private boolean hasElement(double inDist, int iteration)
    {
        int count = 0;
        for (int i = 0; i < iteration; i++)
        {
            double dist = getDistance();
            if (dist > inDist)
            {
                count +=1;
            }
        }
        double percent = count/iteration;
        return percent>0.8 ;
    }
	
	//move around current location to try to find element
	private boolean oneLocation(double inDist, int iteration)
	{
		boolean has = false;
		if (hasElement(inDist, iteration) == true)
    	{
    		has = true;
    	}
    	else
    	{
    		goAngle(50, 90, 50); //needs adjusting 1(power) and 3(duration)
    		if (hasElement(inDist, iteration) ==true)
    		{
    			has = true;
    		}
    		else
    		{
    			goAngle(50, -90, 100);
    			if (hasElement(inDist, iteration) ==true)
        		{
        			has = true;
        		}
    		}
    	}
		return has
	}
    
	//find team element with distance sensor
    private int barcode(double inDist, int iteration)
    {
    	int bar = 0;
    	//move to bar1
    	//motion code here
    	//at bar1
    	if (oneLocation(double inDist, int iteration) == true)
    	{
    		bar = 1;
    		telemetry.addLine("Found at bar 1.");
    		telemetry.update();
    	}
    	else
    	{
    		//move to bar2
    		//motion code here
    		//at bar2
    		if (oneLocation(double inDist, int iteration) == true)
        	{
        		bar = 2;
        		telemetry.addLine("Found at bar 2.");
        		telemetry.update();
        	}
    		else
    		{
    			//automatically assume element is at bar3
    			bar = 3;
        		telemetry.addLine("Found at bar 3.");
        		telemetry.update()；
    		}
    	}
        return bar;
    }
}
