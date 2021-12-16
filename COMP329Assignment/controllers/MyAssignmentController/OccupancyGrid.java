// File: OccupancyGrid.java
// Date: 20 Nov 2021
// Description: OccupancyGrid Class support for COMP329 Programming Assignment (2021)
// Author: Terry Payne
// Modifications:
/**
 *
 * @author Dr Terry R. Payne (trp@liv.ac.uk)
 *
 */
 
import com.cyberbotics.webots.controller.DistanceSensor;
import com.cyberbotics.webots.controller.Display;

 
 public class OccupancyGrid {
  private double arenaHeight;    // Height of the arena
  private double arenaWidth;     // Width of the arena
  private double cellWidth;      // Width of a cell (wrt arena)
  private double cellHeight;     // Height of a cell (wrt arena)
  private int numCellsInRow;	    // number of cells across
  private int numCellsInCol;	    // number of cells up
  private DistanceSensor[] ps;   // array of distance sensors attached to the robot
  private Pose[] psPose;         // the pose of each sensor (assuming the robot is a round cylinder)
  private double[] grid;         // Array of log odds elements
  private double maxRange;       // we'll get this from the lookup table of so0
  private double maxValue;       // we'll get this from the parameters of so0
  private double radius;         // radius of the robot (assume it is round)
  private Pose robotPose;        // track the pose of the robot in the global coordinate system

  // Fixed log odds values (found empirically)  
  private final double lprior = Math.log(0.5/(1-0.5));
  private final double locc = Math.log(0.95/(1-0.95));
  private final double lfree = Math.log(0.45/(1-0.45));		

  // Constants for the inverse sensor model (values are halved to optimise performance)
  private final double HALFALPHA = 0.04;           // Thickness of any wall found
  private final double HALFBETA = Math.PI/36.0;    // sensor cone opening angle 



  // ==================================================================================
  // Constructor
  // ==================================================================================
  public OccupancyGrid(double arenaHeight,
                       double arenaWidth,
                       int numCellsInCol,
                       int numCellsInRow,
                       DistanceSensor[] ps,
                       double[] psAngleDeg,
                       double radius) {
                       
    // Store instance values
    this.arenaHeight = arenaHeight;
    this.arenaWidth = arenaWidth;
    this.numCellsInRow = numCellsInRow;
    this.numCellsInCol = numCellsInCol;
    this.grid = new double[numCellsInRow*numCellsInCol];
    this.ps = ps;
    this.radius = radius;
    this.robotPose = new Pose();
    
    this.cellWidth = this.arenaWidth / (double) this.numCellsInRow;
    this.cellHeight = this.arenaHeight / (double) this.numCellsInCol;
    
    // ---------------------------------------------------------------------------
    // Initialise grid
    for (int i=0; i < this.grid.length; i++)
      this.grid[i] = lprior;    
      
    // ---------------------------------------------------------------------------
    // Generate some diagnostic information and
    // determine max range from lookup table  
    double[] lt = ps[0].getLookupTable();
    this.maxRange=0.0;                      // start with a minimum value
    for (int i=0; i< lt.length; i++) {
      if ((i%3)==0) this.maxRange=lt[i];
      if ((i%3)==2) System.out.println("\n");
    }
    this.maxValue=ps[0].getMaxValue();

    // ---------------------------------------------------------------------------
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
  // Getters / Setters  
  // ==================================================================================
  public int getNumCellsInRow() {
    return this.numCellsInRow;
  }
  public int getNumCellsInCol() {
    return this.numCellsInCol;
  }
  public int getGridSize() {
    return this.grid.length;
  }
  public double getCellProbability(int index) {
    return getProbFromLogOdds(grid[index]);
  }

  
  // ==================================================================================
  // Internal Methods  
  // ==================================================================================

  private double getProbFromLogOdds(double lodds) {
    return 1-(1/(1+Math.exp(lodds)));
  }
  
  private double getSensorReading(int k) {
    return this.maxRange - (this.maxRange/this.maxValue * ps[k].getValue());
  }
  
  private double invSensorModel(Pose p, double x, double y) {

    // ---------------------------------------------------------------------------
    // Determine the range and bearing of the cell
    double deltaX = x-p.getX();
    double deltaY = y-p.getY();
    double r = Math.sqrt(Math.pow(deltaX,2)+Math.pow(deltaY,2));     // range
    double phi = Math.atan2(deltaY, deltaX) - p.getTheta();          // bearing
    int k = 0;                                                       // sensor index
    double kDelta;
    double logodds = lprior;                                         // default return value

    // Note that the above code assumes range based
    // on the robot and not the sensor position
    if (r>this.radius)
      r=r-this.radius;    // Remove the distance from the robot center to sensor
    else  
      r=0.0;              // If negative, then cell center is behind the sensor and
                          // within the robot radiuis.  So just reset to zero.
    // ---------------------------------------------------------------------------
    // Find the nearest sensor to the cell
    // Initialise the angle to be PI as this all other angles will be less (clockwise or anticlockwise)
    double kMinDelta=Math.PI; // Smallest distance away from a sensor (we try to minimise this)
        
    for (int j=0; j<ps.length; j++) {
      kDelta = Math.abs(psPose[j].getDeltaTheta(phi));
      if (kDelta < kMinDelta) {
        k=j;
        kMinDelta = kDelta;
      }
    }
    // we now known that k=closest sensor, and kMinDelta is the difference in angle
    
    // ---------------------------------------------------------------------------
    // Determine which region the cell is in
    double z = getSensorReading(k);
    if (z == this.maxRange) {
      logodds = lprior;
    } else if ((r > Math.min(this.maxRange, z+HALFALPHA)) || (kMinDelta > HALFBETA)) {
      // Region 3 - (unknown)
      logodds = lprior;
    } else if ((z < this.maxRange) && (Math.abs(r-z) < HALFALPHA)) {
      // Region 1 - (occupied)
      logodds = locc;
    } else if (r <= z) {
      // Region 1 - (free)
      logodds = lfree;
    }
    return logodds;
  }

  // ==================================================================================
  // External Methods  
  // ==================================================================================
  public void occupancy_grid_mapping(Pose p) {
    
    double x;                              // x coord
    double y;                              // y coord
    double xOrigOffset = arenaWidth/2.0;   // Offset for origin along x axis
    double yOrigOffset = arenaHeight/2.0;  // Offset for origin along y axis
    
    double xInc = arenaWidth/numCellsInRow;
    double yInc = arenaHeight/numCellsInCol;

    double xCellOffset = xInc/2.0;          // offset to center of cell along x axis
    double yCellOffset = yInc/2.0;          // offset to center of cell along y axis
    
    
    for (int i=0; i < this.grid.length; i++) {
      // Convert cell into a coordinate.  Recall that the arena is dimensions -n..+n
      x = xInc * (i%numCellsInRow)-xOrigOffset+xCellOffset;
      y = -(yInc * (i/numCellsInRow)-yOrigOffset+yCellOffset);
      
      // Log Odds Update Function
      this.grid[i] = this.grid[i] + invSensorModel(p, x, y) - lprior;
    }
  } 
}