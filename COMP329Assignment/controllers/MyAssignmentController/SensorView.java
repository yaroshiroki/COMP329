// File: SensorView.java
// Date: 28th Oct 2021
// Description: Represent the local area of the adept robot given its sonar sensors
// Author: Terry Payne
// Modifications:
//     Updated for the Programming Assignment 2021 (29th Nov 20201)


/**
 * This class displays the robot's internal model of the environment as a set of
 * sensor rays, based on the returned values of each sensor.
 * Ideally this should have been a subclass of the Display class, but it is easier
 * for this to take a reference to a Display object and use this to render the graphic.
 * 
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;

public class SensorView {
  private Display display;      // reference to the display device on the robot
  private DistanceSensor[] ps;  // array of distance sensors attached to the robot
  private Pose[] psPose;        // the pose of each sensor (assuming the robot is a round cylinder)
  private Pose robotPose;       // we only track the orientation, as the robot is always centered
  
  private double maxRange;      // we'll get this from the lookup table of so0
  private double maxValue;      // we'll get this from the parameters of so0
  private double radius;        // radius of the robot (assume it is round)
  private int deviceWidth;      // width of the display device
  private int deviceHeight;     // height of the display device
    
  private double scaleFactor;   // Scale factor to scale rendered map to the maximal dimension on the display
	
  // Colours used by the display
  private final static int DARKGREY = 0x3C3C3C;
  private final static int GREY = 0x787878;
  private final static int BLACK = 0x000000;
  private final static int WHITE = 0xFFFFFF;

  // ==================================================================================
  // Constructors
  // ==================================================================================
  public SensorView(Display d, DistanceSensor[] ps, double[] psAngleDeg, double radius) {
    this.display = d;
    this.ps = ps;
    this.radius = radius;
    this.deviceWidth = this.display.getWidth();
    this.deviceHeight = this.display.getHeight();
    this.robotPose = new Pose();
    
    //----------------------------
    // Generate some diagnostic information and
    // determine max range from lookup table  
    double[] lt = ps[0].getLookupTable();
    //System.out.println("Lookup Table has "+lt.length+" entries");
    this.maxRange=0.0;                      // start with a minimum value
    for (int i=0; i< lt.length; i++) {
      if ((i%3)==0) this.maxRange=lt[i];
      //System.out.print(" "+lt[i]+",");
      if ((i%3)==2) System.out.println("\n");
    }
    this.maxValue=ps[0].getMaxValue();
//    System.out.println("Max Range: "+this.maxRange);
//    System.out.println("Robot Diameter: "+this.radius*2);
    
    // Now we can scale to the display
    // Determine the rendering scale factor
    this.scaleFactor = Math.min(this.deviceWidth, this.deviceHeight)/(2*(this.maxRange+this.radius));   

    // Determine the pose (relative to the robot of each of the sensors)
    this.psPose = new Pose[psAngleDeg.length];    // Allocate the pose array
    for (int i=0; i< psAngleDeg.length; i++) {
      double theta = Math.toRadians(psAngleDeg[i]);
      this.psPose[i] = new Pose(Math.cos(theta)*this.radius,
                                Math.sin(theta)*this.radius,
                                theta);      
    }
  }
  
  // ==================================================================================
  // Internal (Private) methods
  // ==================================================================================
  // Map the real coordinates to screen coordinates assuming
  // the origin is in the center and y axis is inverted
  private int scale(double l) {
    return (int) (this.scaleFactor * l);
  }
  private int mapX(double x) {
    return (int) ((deviceWidth/2.0) + scale(x));
  }
  private int mapY(double y) {
    return (int) ((deviceHeight/2.0) - scale(y));
  }
  private double rotX(double x, double y, double theta) {
    return Math.cos(theta)*x - Math.sin(theta)*y;
  }
  private double rotY(double x, double y, double theta) {
    return Math.sin(theta)*x + Math.cos(theta)*y;
  }

  // ==================================================================================
  // External (Public) methods
  // ==================================================================================  
  public void setPose(Pose p) {
    this.robotPose.setPosition(p);
  } 
  
  public void paintView() {
  
    // ===================================================================================
    // draw a background
    this.display.setColor(0xF0F0F0);     // Off White
    this.display.fillRectangle(0, 0, this.deviceWidth, this.deviceHeight);
 
    // ===================================================================================
    // Draw the robot in the centor of the display
    // Assume full coordinate system is in double coords, and then only map to screen
    // including scaling when drawing
    
    int robotX = (int) (deviceWidth/2.0);
    int robotY = (int) (deviceWidth/2.0);
    
    int robotR = (int) (this.radius * this.scaleFactor);  // body radius
    double robotH = this.robotPose.getTheta(); 

    // ===================================================================================
    // Draw Robot Body          
    this.display.setColor(WHITE);     // White
    this.display.fillOval(mapX(0.0), mapY(0.0), scale(this.radius), scale(this.radius));
    this.display.setColor(DARKGREY);     // Dark Grey
    this.display.drawOval(mapX(0.0), mapY(0.0), scale(this.radius), scale(this.radius));

    // ===================================================================================
    // Need to indicate heading          
    double headingX = Math.cos(robotH) * this.radius;
    double headingY = Math.sin(robotH) * this.radius;
    this.display.setColor(DARKGREY);     // Dark Grey
    this.display.drawLine(mapX(0.0), mapY(0.0), mapX(headingX), mapY(headingY));

    // Base sensor values on sensor so0
    this.display.setColor(BLACK);         // Black
    this.display.setFont("Arial", 8, true);  // font size = 8, with antialiasing
    
    // For each sensor, get the value of the sensor and display the distance
    // Note that we assume a array of four values.  Each polygon only requires 
    // three, but the fourthi is used to determine a posion for the text
    int[] xArc = {0, 0, 0, 0};
    int[] yArc = {0, 0, 0, 0};
    double[] xd = {0.0, 0.0, 0.0, 0.0};
    double[] yd = {0.0, 0.0, 0.0, 0.0};
    double d;      // distance measured

    
    // Draw triangles of 2*Math.PI/36.0 either side either side of the sensor orientation
    for (int i = 0; i < ps.length ; i++) {

      d = this.maxRange - (this.maxRange/this.maxValue * ps[i].getValue());

      xd[0] = psPose[i].getX();
      xd[1] = psPose[i].getX() + (d * Math.cos(psPose[i].getTheta()-(Math.PI/18.0)));
      xd[2] = psPose[i].getX() + (d * Math.cos(psPose[i].getTheta()+(Math.PI/18.0)));
      xd[3] = psPose[i].getX() + ((d+0.3) * Math.cos(psPose[i].getTheta()));
      
      yd[0] = psPose[i].getY();
      yd[1] = psPose[i].getY() + (d * Math.sin(psPose[i].getTheta()-(Math.PI/18.0)));
      yd[2] = psPose[i].getY() + (d * Math.sin(psPose[i].getTheta()+(Math.PI/18.0)));
      yd[3] = psPose[i].getY() + ((d+0.3) * Math.sin(psPose[i].getTheta()));
      
      // Need to rotate each point using the rotation matrix and the robot orientation
      for (int j=0; j<4; j++) {
        xArc[j] = mapX(rotX(xd[j], yd[j], robotH));
        yArc[j] = mapY(rotY(xd[j], yd[j], robotH));
      } 
      
      // Only use the first three points for the polygon
      this.display.fillPolygon(xArc, yArc, 3);
      // Use the fourth point for the distance string, with a slight offset as the
      // coordinates are for the location of the bottom left of the string
      this.display.drawText(String.format("%.02f", d), xArc[3]-10, yArc[3]);
    } 

    // Provide information on the actual pose of the robot
    this.display.setFont("Arial", 10, true);  // font size = 10, with antialiasing
    this.display.drawText("Pose", 1,470);
    this.display.drawText(robotPose.toString(), 1,486);
  }   
}
