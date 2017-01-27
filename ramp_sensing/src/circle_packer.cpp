#include "circle_packer.h"
#include "Utilties.cpp"



CirclePacker::CirclePacker(nav_msgs::OccupancyGridConstPtr g)
{
  //ROS_INFO("In CirclePacker::CirclePacker()");
  grid_ = *g;
  convertOGtoMat(g);
}

CirclePacker::~CirclePacker() {}



int CirclePacker::CircleFitByLevenbergMarquardtFull (Data& data, CircleFit& circleIni, reals LambdaIni, CircleFit& circle)
/*                                     <------------------ Input ------------------->  <-- Output -->

       Geometric circle fit to a given set of data points (in 2D)
    
       Input:  data     - the class of data (contains the given points):
    
         data.n   - the number of data points
         data.X[] - the array of X-coordinates
         data.Y[] - the array of Y-coordinates
              
               circleIni - parameters of the initial circle ("initial guess")
            
         circleIni.a - the X-coordinate of the center of the initial circle
         circleIni.b - the Y-coordinate of the center of the initial circle
         circleIni.r - the radius of the initial circle
            
         LambdaIni - the initial value of the control parameter "lambda"
                     for the Levenberg-Marquardt procedure
                     (common choice is a small positive number, e.g. 0.001)
            
       Output:
         integer function value is a code:
                    0:  normal termination, the best fitting circle is 
                        successfully found
                    1:  the number of outer iterations exceeds the limit (99)
                        (indicator of a possible divergence)
                    2:  the number of inner iterations exceeds the limit (99)
                        (another indicator of a possible divergence)
                    3:  the coordinates of the center are too large
                        (a strong indicator of divergence)
                       
         circle - parameters of the fitting circle ("best fit")
            
         circle.a - the X-coordinate of the center of the fitting circle
         circle.b - the Y-coordinate of the center of the fitting circle
         circle.r - the radius of the fitting circle
         circle.s - the root mean square error (the estimate of sigma)
         circle.i - the total number of outer iterations (updating the parameters)
         circle.j - the total number of inner iterations (adjusting lambda)
            
       Algorithm:  Levenberg-Marquardt running over the full parameter space (a,b,r)
                         
       See a detailed description in Section 4.5 of the book by Nikolai Chernov:
       "Circular and linear regression: Fitting circles and lines by least squares"
       Chapman & Hall/CRC, Monographs on Statistics and Applied Probability, volume 117, 2010.
         
    Nikolai Chernov,  February 2014
*/
{
    int code,i,iter,inner,IterMAX=99;
    
    reals factorUp=10.,factorDown=0.04,lambda,ParLimit=1.e+6;
    reals dx,dy,ri,u,v;
    reals Mu,Mv,Muu,Mvv,Muv,Mr,UUl,VVl,Nl,F1,F2,F3,dX,dY,dR;
    reals epsilon=3.e-8;
    reals G11,G22,G33,G12,G13,G23,D1,D2,D3;
    
    CircleFit Old,New;
    
//       starting with the given initial circle (initial guess)
    
    New = circleIni;
    
//       compute the root-mean-square error via function Sigma; see Utilities.cpp

    New.s = Sigma(data,New);
    
//       initializing lambda, iteration counters, and the exit code
    
    lambda = LambdaIni;
    iter = inner = code = 0;
    
NextIteration:
  
    Old = New;
    if (++iter > IterMAX) {code = 1;  goto enough;}
    
//       computing moments
    
    Mu=Mv=Muu=Mvv=Muv=Mr=0.;
    
    for (i=0; i<data.n; i++)
    {
        dx = data.X[i] - Old.a;
        dy = data.Y[i] - Old.b;
        ri = sqrt(dx*dx + dy*dy);
        u = dx/ri;
        v = dy/ri;
        Mu += u;
        Mv += v;
        Muu += u*u;
        Mvv += v*v;
        Muv += u*v;
        Mr += ri;
    }
    Mu /= data.n;
    Mv /= data.n;
    Muu /= data.n;
    Mvv /= data.n;
    Muv /= data.n;
    Mr /= data.n;
    
//       computing matrices
    
    F1 = Old.a + Old.r*Mu - data.meanX;
    F2 = Old.b + Old.r*Mv - data.meanY;
    F3 = Old.r - Mr;
    
    Old.g = New.g = sqrt(F1*F1 + F2*F2 + F3*F3);
    
try_again:
  
    UUl = Muu + lambda;
    VVl = Mvv + lambda;
    Nl = One + lambda;
    
//         Cholesly decomposition
    
    G11 = sqrt(UUl);
    G12 = Muv/G11;
    G13 = Mu/G11;
    G22 = sqrt(VVl - G12*G12);
    G23 = (Mv - G12*G13)/G22;
    G33 = sqrt(Nl - G13*G13 - G23*G23);
    
    D1 = F1/G11;
    D2 = (F2 - G12*D1)/G22;
    D3 = (F3 - G13*D1 - G23*D2)/G33;
    
    dR = D3/G33;
    dY = (D2 - G23*dR)/G22;
    dX = (D1 - G12*dY - G13*dR)/G11;
    
    if ((abs(dR)+abs(dX)+abs(dY))/(One+Old.r) < epsilon) goto enough;
    
//       updating the parameters
    
    New.a = Old.a - dX;
    New.b = Old.b - dY;
    
    if (abs(New.a)>ParLimit || abs(New.b)>ParLimit) {code = 3; goto enough;}
    
    New.r = Old.r - dR;
    
    if (New.r <= 0.)
    {
        lambda *= factorUp;
        if (++inner > IterMAX) {code = 2;  goto enough;}
        goto try_again;
    }
    
//       compute the root-mean-square error via function Sigma; see Utilities.cpp

    New.s = Sigma(data,New);   
    
//       check if improvement is gained
    
    if (New.s < Old.s)    //   yes, improvement
    {
        lambda *= factorDown;
        goto NextIteration;
    }
    else                       //   no improvement
    {
        if (++inner > IterMAX) {code = 2;  goto enough;}
        lambda *= factorUp;
        goto try_again;
    }
    
    //       exit
    
enough:
  
    Old.i = iter;    // total number of outer iterations (updating the parameters)
    Old.j = inner;   // total number of inner iterations (adjusting lambda)
    
    circle = Old;
    
    return code;
}







CircleFit CirclePacker::CircleFitFitByTaubin (Data& data)
/*  
      CircleFit fit to a given set of data points (in 2D)
      
      This is an algebraic fit, due to Taubin, based on the journal article
     
      G. Taubin, "Estimation Of Planar Curves, Surfaces And Nonplanar
                  Space Curves Defined By Implicit Equations, With 
                  Applications To Edge And Range Image Segmentation",
                  IEEE Trans. PAMI, Vol. 13, pages 1115-1138, (1991)

      Input:  data     - the class of data (contains the given points):
    
        data.n   - the number of data points
        data.X[] - the array of X-coordinates
        data.Y[] - the array of Y-coordinates
     
     Output:	       
               circle - parameters of the fitting circle:
            
         circle.a - the X-coordinate of the center of the fitting circle
         circle.b - the Y-coordinate of the center of the fitting circle
         circle.r - the radius of the fitting circle
         circle.s - the root mean square error (the estimate of sigma)
         circle.j - the total number of iterations
             
     The method is based on the minimization of the function
     
                  sum [(x-a)^2 + (y-b)^2 - R^2]^2
              F = -------------------------------
                      sum [(x-a)^2 + (y-b)^2]
                 
     This method is more balanced than the simple Kasa fit.
        
     It works well whether data points are sampled along an entire circle or
     along a small arc. 
     
     It still has a small bias and its statistical accuracy is slightly
     lower than that of the geometric fit (minimizing geometric distances),
     but slightly higher than that of the very similar Pratt fit. 
     Besides, the Taubin fit is slightly simpler than the Pratt fit
     
     It provides a very good initial guess for a subsequent geometric fit. 
     
       Nikolai Chernov  (September 2012)

*/
{
    int i,iter,IterMAX=99;
    
    reals Xi,Yi,Zi;
    reals Mz,Mxy,Mxx,Myy,Mxz,Myz,Mzz,Cov_xy,Var_z;
    reals A0,A1,A2,A22,A3,A33;
    reals Dy,xnew,x,ynew,y;
    reals DET,Xcenter,Ycenter;
    
    CircleFit circle;
    
    data.means();   // Compute x- and y- sample means (via a function in the class "data")

//     computing moments 

  Mxx=Myy=Mxy=Mxz=Myz=Mzz=0.;
    
    for (i=0; i<data.n; i++)
    {
        Xi = data.X[i] - data.meanX;   //  centered x-coordinates
        Yi = data.Y[i] - data.meanY;   //  centered y-coordinates
        Zi = Xi*Xi + Yi*Yi;
        
        Mxy += Xi*Yi;
        Mxx += Xi*Xi;
        Myy += Yi*Yi;
        Mxz += Xi*Zi;
        Myz += Yi*Zi;
        Mzz += Zi*Zi;
    }
    Mxx /= data.n;
    Myy /= data.n;
    Mxy /= data.n;
    Mxz /= data.n;
    Myz /= data.n;
    Mzz /= data.n;
    
//      computing coefficients of the characteristic polynomial
    
    Mz = Mxx + Myy;
    Cov_xy = Mxx*Myy - Mxy*Mxy;
    Var_z = Mzz - Mz*Mz;
    A3 = Four*Mz;
    A2 = -Three*Mz*Mz - Mzz;
    A1 = Var_z*Mz + Four*Cov_xy*Mz - Mxz*Mxz - Myz*Myz;
    A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy;
    A22 = A2 + A2;
    A33 = A3 + A3 + A3;

//    finding the root of the characteristic polynomial
//    using Newton's method starting at x=0  
//     (it is guaranteed to converge to the right root)
    
    for (x=0.,y=A0,iter=0; iter<IterMAX; iter++)  // usually, 4-6 iterations are enough
    {
          Dy = A1 + x*(A22 + A33*x);
        xnew = x - y/Dy;
        if ((xnew == x)||(!std::isfinite(xnew))) break;
        ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3));
        if (abs(ynew)>=abs(y))  break;
        x = xnew;  y = ynew;
    }
     
//       computing paramters of the fitting circle
    
    DET = x*x - x*Mz + Cov_xy;
    Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/DET/Two;
    Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/DET/Two;

//       assembling the output

    circle.a = Xcenter + data.meanX;
    circle.b = Ycenter + data.meanY;
    circle.r = sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz);
    circle.s = Sigma(data,circle);
    circle.i = 0;
    circle.j = iter;  //  return the number of iterations, too
    
    return circle;
}













void CirclePacker::convertOGtoMat(nav_msgs::OccupancyGridConstPtr g)
{
  //ROS_INFO("In CirclePacker::convertOGtoMat");

  
  // Use the GridMap2D library to convert from nav_msgs::OccupancyGrid to cv::Mat
  gridmap_2d::GridMap2D gmap(g, false);

  // Create a window
  //cv::namedWindow("testing", CV_WINDOW_AUTOSIZE);

  src = gmap.binaryMap();

  // Show the image
  //cv::imshow("testing", src);

  // PRESS ESC TO BEFORE CLOSING WINDOW, OTHERWISE THE PROGRAM WILL HANG
  //cv::waitKey(0);
}

void CirclePacker::CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src, detected_edges, cv::Size(3,3) );

  //std::cout<<"\nDetected Edges: "<<detected_edges;
  //ROS_INFO("detected_edges type: %i", detected_edges.type());
  //ROS_INFO("lowThreshold: %i", lowThreshold);

  // Somehow, lowThreshold is being converted to unsigned int before this point
  // its value is 32767 (-1 for unsigned 4-byte int)
  // Set the value back to 0 for edge detection to work
  lowThreshold = 0;

  /// Canny detector
  cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = cv::Scalar::all(0);

  //std::cout<<"\nDetected Edges: "<<detected_edges;

  src.copyTo( dst, detected_edges);
  //cv::imshow("detected_edges", dst);
  //cv::waitKey(0);
  //imshow( window_name, dst );
}

double CirclePacker::getMinDistToPoly(const Polygon& poly, const Cell& cell)
{
  double result = 100000;

  for(int n=0;n<poly.normals.size();n++)
  {
    //std::cout<<"\npoly.normals[n]: "<<poly.normals[n].a<<", "<<poly.normals[n].b;
    // Get unit normal
    double l = sqrt( pow(poly.normals[n].a,2) + pow(poly.normals[n].b,2) );
    Normal v_hat;
    v_hat.a = poly.normals[n].a / l;
    v_hat.b = poly.normals[n].b / l;
    
    std::vector<double> r;
    r.push_back(poly.edges[n].start.x - cell.p.x);
    r.push_back(poly.edges[n].start.y - cell.p.y);

    double d = fabs((v_hat.a*r[0]) + (v_hat.b*r[1]));
    //std::cout<<"\nl: "<<l<<" v_hat: <"<<v_hat.a<<","<<v_hat.b<<"> r: <"<<r[0]<<","<<r[1]<<">";
    //std::cout<<"\nd: "<<d;
    if(d < result)
    {
      result = d;
    }
  }

  return result;
}

double CirclePacker::getMinDistToCirs(const std::vector<Circle>& cirs, const Cell& cell)
{
  if(cirs.size() == 0)
  {
    return -1;
  }

  double result=10000;

  for(int i=0;i<cirs.size();i++)
  {
    double dist = sqrt( pow( cell.p.x - cirs[i].center.x, 2) + pow( cell.p.y - cirs[i].center.y, 2) );
    
    // Then, subtract the radius to get the dist to the outside of the circle
    dist -= cirs[i].radius;

    if( dist < result)
    {
      result = dist;
    }
  }

  return result;
}


void CirclePacker::deleteCellsInCir(const std::vector<Cell>& cells, const Circle cir, std::vector<Cell>& result)
{
  //std::cout<<"\nIn deleteCellsInCir\n";
  for(int i=0;i<cells.size();i++)
  {
    //std::cout<<"\nTesting cell "<<cells[i].p.x<<", "<<cells[i].p.y;
    //std::cout<<"\nDist: "<<(sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ));
    // Get distance between circle of cell and new cell, check if > its radius
    if( sqrt( pow( cir.center.x - cells[i].p.x, 2) + pow( cir.center.y - cells[i].p.y, 2) ) > cir.radius )
    {
      result.push_back(cells[i]);
    }
  }
  //std::cout<<"\nExiting deleteCellsInCir\n";
}


Normal CirclePacker::computeNormal(Edge e)
{
  //std::cout<<"\ne.start: "<<e.start.x<<" "<<e.start.y;
  //std::cout<<"\ne.end: "<<e.end.x<<" "<<e.end.y;
  Normal result;
  result.a = e.end.y - e.start.y;
  result.b = -(e.end.x - e.start.x);

  result.c = -((result.a*e.start.x) + (result.b*e.start.y));

  //std::cout<<"\na: "<<result.a<<" b: "<<result.b<<" c: "<<result.c;
  return result;
}


bool CirclePacker::cellInPoly(Polygon poly, cv::Point cell)
{
  for(int i=0;i<poly.normals.size();i++)
  {
    //std::cout<<"\nnormal a: "<<poly.normals[i].a<<" b: "<<poly.normals[i].b<<" c: "<<poly.normals[i].c;
    double d = poly.normals[i].a*cell.x + poly.normals[i].b*cell.y + poly.normals[i].c;
    //std::cout<<"\ncell center: "<<cell.x<<", "<<cell.y<<" d: "<<d;
    if(d > -0.000001)
    {
      //std::cout<<"\nNot in polygon";
      return false;
    }
  }

  return true;
}


std::vector<Circle> CirclePacker::getCirclesFromPoly(Polygon poly)
{
  //std::cout<<"\n# of edges: "<<poly.edges.size();
  std::vector<Circle> result;
  std::vector<cv::Point> vertices;
  
  // Push on all vertices
  for(int i=0;i<poly.edges.size();i++)
  {
    vertices.push_back(poly.edges[i].start);
    vertices.push_back(poly.edges[i].end);
  }
  
  double MAX_LENGTH= vertices[0].y;
  double MAX_WIDTH = vertices[0].x;
  double MIN_LENGTH= vertices[0].y;
  double MIN_WIDTH = vertices[0].x;
  for(int i=0;i<vertices.size();i++)
  {
    if(vertices[i].y > MAX_LENGTH)
    {
      MAX_LENGTH = vertices[i].y;
    }
    if(vertices[i].y < MIN_LENGTH)
    {
      MIN_LENGTH = vertices[i].y;
    }
    
    if(vertices[i].x > MAX_WIDTH)
    {
      MAX_WIDTH = vertices[i].x;
    }
    if(vertices[i].x < MIN_WIDTH)
    {
      MIN_WIDTH = vertices[i].x;
    }
  }

  double round = 1;
  int width_count = (MAX_WIDTH - MIN_WIDTH) / round;
  int length_count = (MAX_LENGTH - MIN_LENGTH) / round;
  double start_x = MIN_WIDTH + round/2.f;
  double start_y = MIN_LENGTH + round/2.f;

  //std::cout<<"\nMAX_WIDTH: "<<MAX_WIDTH<<" MAX_LENGTH: "<<MAX_LENGTH<<" width_count: "<<width_count<<" length_count: "<<length_count;

  std::vector<Cell> cells;
 
  for(int i=0;i<width_count;i++)
  {
    for(int j=0;j<length_count;j++)
    {
      //std::cout<<"\ni: "<<i<<" j: "<<j<<" round: "<<round;
      double x = start_x + (round * (i)); 
      double y = start_y + (round * (j));
      Cell temp;
      temp.p.x = x;
      temp.p.y = y;
    
      //std::cout<<"\n("<<temp.p.x<<", "<<temp.p.y<<")";

      if(cellInPoly(poly, temp.p))
      {
        cells.push_back(temp);
      }
    }
  }
  

  std::vector<Cell> reduced_cells = cells;


  while(cells.size() > 0)
  {
    //std::cout<<"\nIn while cells.size(): "<<cells.size()<<" result.size(): "<<result.size();
    cells = reduced_cells;

    std::priority_queue<Cell, std::vector<Cell>, CompareDist> updated_pq;

    // Delete all cells whose centers lie in the largest circle
    if(result.size() > 0)
    {
      reduced_cells.clear();
      deleteCellsInCir(cells, result[result.size()-1], reduced_cells);
    }

    // Recalculate the distance, include existing circles!
    // For each cell, compute distance to the closest polygon edge
    for(int i=0;i<reduced_cells.size();i++)
    {
      Cell& cell = reduced_cells[i];

      double min_d=getMinDistToPoly(poly, cell);
      double min_cir=getMinDistToCirs(result,cell);

      if(min_d < min_cir || min_cir < 0)
      {
        cell.dist = min_d;
      }
      else
      {
        cell.dist = min_cir;
      }

      updated_pq.push(cell);
    } // end for each cell

    if(!updated_pq.empty())
    {
      Cell c = updated_pq.top();
      Circle temp;

      temp.center.x = c.p.x;
      temp.center.y = c.p.y;
      temp.radius = c.dist;

      result.push_back(temp);
    }
  }
  
  /*std::cout<<"\nFinal number of circles: "<<result.size();
  for(int i=0;i<result.size();i++)
  {
    std::cout<<"\nCircle "<<i<<" ("<<result[i].center.x<<", "<<result[i].center.y<<") radius: "<<result[i].radius;
  }*/

  return result;
}


std::vector<Triangle> CirclePacker::triangulatePolygon(const Polygon& poly)
{
  std::vector<Triangle> result;

  // Get all vertices of polygon
  std::vector<cv::Point> vertices;
  for(int i=0;i<poly.edges.size();i++)
  {
    vertices.push_back(poly.edges[i].start);
  }
  
  std::vector<int> i_reflex;

  // For each vertex, get its two neighbors
  // Check if line connecting them is in polygon
  for(int i=0;i<vertices.size();i++)
  {
    std::cout<<"\nVertex "<<i<<": ("<<vertices[i].x<<", "<<vertices[i].y<<")";

    // Get neighbors
    cv::Point v0 = i == 0 ? vertices[vertices.size()-2] : vertices[i-1];
    cv::Point v1 = vertices[i];
    cv::Point v2 = i == vertices.size() - 1 ? vertices[0] : vertices[i+1];

    std::cout<<"\nNeighbors: ("<<v0.x<<", "<<v0.y<<") ("<<v2.x<<", "<<v2.y<<")";
   
    // Get direction angle of each segment
    // y component negated because grid y increases as it goes down
    double ax = v0.x - v1.x;
    double bx = v2.x - v1.x;
    double ay = -( v0.y - v1.y );
    double by = -( v2.y - v1.y );
    std::cout<<"\nax: "<<ax<<" ay: "<<ay<<" bx: "<<bx<<" by: "<<by;

    double ta = atan2(ay , ax);
    double tb = atan2(by , bx);
    std::cout<<"\nta: "<<ta<<" tb: "<<tb;
   
    double t_final = fmodf(ta - tb, 6.28);
    std::cout<<"\nt_final: "<<t_final;

    if(t_final > 3.14159)
    {
      i_reflex.push_back(i);
    }
  } // end for each vertex


  return result;
}


void CirclePacker::combineTwoCircles(const Circle a, const Circle b, Circle& result) const
{
  // Get the midpoint between the arcles
  std::vector<double> vec, midpoint;
  vec.push_back( b.center.x - a.center.x );
  vec.push_back( b.center.y - a.center.y );

  ROS_INFO("vec: [%f, %f]", vec[0], vec[1]);

  midpoint.push_back( a.center.x + (0.5*vec[0]) ); 
  midpoint.push_back( a.center.y + (0.5*vec[1]) ); 

  ROS_INFO("midpoint: (%f, %f)", midpoint[0], midpoint[1]);

  // Get the distance from the midpoint to each center
  double d_mid_i = utility_.positionDistance(midpoint[0], midpoint[1], a.center.x, a.center.y);
  double d_mid_ii = utility_.positionDistance(midpoint[0], midpoint[1], b.center.x, b.center.y);

  double R = d_mid_i > d_mid_ii ? d_mid_i+a.radius : d_mid_ii+b.radius;

  ROS_INFO("d_mid_i: %f d_mid_ii: %f R: %f", d_mid_i, d_mid_ii, R);

  result.center.x = midpoint[0];
  result.center.y = midpoint[1];
  result.radius = R;
}


// result is a final list of circles: contains both the combined ones and the ones that were not combined
void CirclePacker::combineOverlappingCircles(std::vector<Circle> cs, std::vector<Circle>& result) const
{
  ROS_INFO("In combineOverlappingCircles");
  for(int i=0;i<cs.size();i++)
  {
    ROS_INFO("Circle %i - Center: (%f, %f) Radius: %f", i, cs[i].center.x, cs[i].center.y, cs[i].radius);
  }
  int pairs=0;

  //result = cs;

  int i=0,j=0;

  while(i<cs.size()-1)
  {
    ROS_INFO("i: %i", i);
    ROS_INFO("cs.size(): %i", (int)cs.size());
    Circle ci = cs[i];
    ROS_INFO("ci - Center: (%f, %f) Radius: %f", ci.center.x, ci.center.y, ci.radius);
    j = i+1;
    double inflate = 7.5;
    double threshold = 0.;

    while(j<cs.size())
    {
      ROS_INFO("j: %i", j);

      // Check if they overlap
      Circle cj = cs[j];
      ROS_INFO("cj - Center: (%f, %f) Radius: %f", cj.center.x, cj.center.y, cj.radius);
     
      double max_r = ci.radius > cj.radius ? ci.radius : cj.radius;
      threshold = max_r + inflate;

      double d = utility_.positionDistance(ci.center.x, ci.center.y, cj.center.x, cj.center.y);
      ROS_INFO("d: %f threshold: %f", d, threshold);

      if(d < threshold)
      {
        ROS_INFO("Combining the Circles!");

        // Combine them
        Circle temp;
        combineTwoCircles(ci, cj, temp);
        ROS_INFO("Result - Center: (%f, %f) Radius: %f", temp.center.x, temp.center.y, temp.radius);

        // If combined, replace both circles with overlapping circle
        // Replace i by setting it to temp, erase the circle at j
        result.push_back(temp);
        cs[i] = temp;
        cs.erase(cs.begin()+j, cs.begin()+j+1);
        //result.erase(cs.begin()+j, cs.begin()+j+1);

        // Then, decrement j to get next circle for comparison
        j--;
      }

      j++;
    } // end inner while

    i++;
  } // end outter while

  result = cs;
  ROS_INFO("After Overlapping:");
  /*for(int i=0;i<cs.size();i++)
  {
    ROS_INFO("Circle %i - Center: (%f, %f) Radius: %f", i, cs[i].center.x, cs[i].center.y, cs[i].radius);
    result.push_back(cs[i]);
  }*/
} // End combineOverlappingCircles



std::vector<Circle> CirclePacker::getCirclesFromEdgeSets(const std::vector< std::vector<Edge> > edge_sets)
{
  std::vector<Circle> result;
  //ROS_INFO("In CirclePacker::getCirclesFromEdgeSets");

 

  for(int i=0;i<edge_sets.size();i++)
  {
    ROS_INFO("Edge set %i", i);
    
    std::vector<double> X, Y;
  
    // For each set of edges, find the minimum and maximum values for x and y
    int x_min = edge_sets[i][0].start.x, 
        y_min = edge_sets[i][0].start.y, 
        x_max = x_min, 
        y_max = y_min;
    for(int j=1;j<edge_sets[i].size();j++)
    {
      X.push_back(edge_sets[i][j].start.x);
      Y.push_back(edge_sets[i][j].start.y);


      ROS_INFO("\tEdge %i - start: (%i,%i) end: (%i,%i)", j, edge_sets[i][j].start.y, edge_sets[i][j].start.x, edge_sets[i][j].end.y, edge_sets[i][j].end.x);
      if( edge_sets[i][j].start.x < x_min )
      {
        x_min = edge_sets[i][j].start.x;
      } 
      if( edge_sets[i][j].start.x > x_max )
      {
        x_max = edge_sets[i][j].start.x;
      } 
      if( edge_sets[i][j].start.y < y_min )
      {
        y_min = edge_sets[i][j].start.y;
      } 
      if( edge_sets[i][j].start.y > y_max )
      {
        y_max = edge_sets[i][j].start.y;
      } 
    } // end inner for
    
    // For each edge set, build a Data object
    /*Data d(edge_sets[i].size(), X.data(), Y.data());
    CircleFit cf = CircleFitFitByTaubin(d);
    cf.print();

    CircleFit out;
    CircleFitByLevenbergMarquardtFull(d, cf, 0.1, out);
    out.print();*/
    



    //ROS_INFO("\tx_min: %i x_max: %i y_min: %i y_max: %i", x_min, x_max, y_min, y_max);

    // Get difference between min+max for both x and y
    double x_diff = fabs(x_max - x_min);
    double y_diff = fabs(y_max - y_min);

    //ROS_INFO("\tx_diff: %f y_diff: %f", x_diff, y_diff);

    // Set radius to half of the largest difference (half because difference would be diameter)
    double r = x_diff > y_diff ? x_diff/2.f : y_diff/2.f;

    /*  
     * Find approximate center
     */
    //****************************************************************************
    // FLIP/SWAP X AND Y!!
    // This is necessary because when Gridmap converts between
    // a costmap and a Mat, it flips the costmap values to conform 
    // to the OpenCV image coordinate system 
    // which starts in the top left corner, x points right, y points down
    //****************************************************************************
    Point cen; 
    cen.x = (y_max + y_min) / 2.f;
    cen.y = (x_max + x_min) / 2.f;

    // Translate the center by 0.075cm in both directions
    cen.x+=1.5;
    cen.y+=1.5;

    ROS_INFO("\tCenter: (%f,%f) Radius: %f", cen.x, cen.y, r);

    Circle temp;

    // Inflate radius by 20cm
    temp.radius = r+4;
    temp.center = cen;

    result.push_back(temp);
  } // end outter for
  
  return result;
}

std::vector<Circle> CirclePacker::getCirclesFromEdges(const std::vector<Edge> edges, const cv::Point robot_cen)
{
  std::vector<Circle> result;

  for(int i=0;i<edges.size();i++)
  {
    Circle temp;

    //ROS_INFO("Edge endpoints: (%i,%i) (%i,%i)", edges[i].start.x, edges[i].start.y, edges[i].end.x, edges[i].end.y);

    // Get length of edge to use as diameter of circle
    double dist = sqrt( pow(edges[i].end.x - edges[i].start.x, 2) + pow(edges[i].end.y - edges[i].start.y, 2) );
    temp.radius = dist/2.f;
    
    // Get the midpoint of the edge
    double x_mid = (edges[i].end.x + edges[i].start.x) / 2.f;
    double y_mid = (edges[i].end.y + edges[i].start.y) / 2.f;
    
    // Get angle between robot center and edge midpoint
    std::vector<double> rob_cen; 
    rob_cen.push_back(robot_cen.x); 
    rob_cen.push_back(robot_cen.y);
    std::vector<double> edge_mid;
    edge_mid.push_back(x_mid);
    edge_mid.push_back(y_mid);
    
    double theta = utility_.findAngleFromAToB(rob_cen, edge_mid); 
    double phi = utility_.displaceAngle(PI, theta);

    // Get circle center with phi
    double psi = utility_.displaceAngle(phi, PI);
    double delta_x = temp.radius*cos(psi);
    double delta_y = temp.radius*sin(psi);

    double x_cen = x_mid + delta_x;
    double y_cen = y_mid + delta_y;

    //ROS_INFO("Edge midpoint: (%f, %f) theta: %f phi: %f psi: %f Circle center: (%f, %f)", x_mid, y_mid, theta, phi, psi, x_cen, y_cen);

    temp.center.x = x_cen;
    temp.center.y = y_cen;

    result.push_back(temp);
  }

  return result;
}


std::vector<Circle> CirclePacker::go()
{
  std::vector<Circle> result;

  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );

  // Convert to grayscale
  //cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  ros::Time t_start_edge_detect = ros::Time::now();
  CannyThreshold(0, 0);
  ros::Duration d_edges_detect(ros::Time::now()-t_start_edge_detect);

  
  // Get the contour points
  ros::Time t_start_contour = ros::Time::now();

  std::vector< std::vector<cv::Point> > detected_contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(detected_edges, detected_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
  
  ros::Duration d_contour(ros::Time::now() - t_start_contour);

  //ROS_INFO("detected_contours size: %i", (int)detected_contours.size());

  /*
   * Get every edge in 1 vector, and make a circle for each edge
   */

  // Make Edges from detected contour points (endpoints of edges)
  std::vector< std::vector<Edge> > edge_sets;
  std::vector<Edge> edges;
  for(int i=0;i<detected_contours.size();i++)
  {
    std::vector<Edge> set;
    //ROS_INFO("detected_contours[%i].size(): %i", i, (int)detected_contours[i].size());
    for(int j=0;j<detected_contours[i].size()-1;j++)
    {
      //ROS_INFO("detected_contours[%i][%i]: (%i, %i)", i, j, detected_contours[i][j].x, detected_contours[i][j].y);
      Edge temp;
      temp.start.x = detected_contours[i][j].x;
      temp.start.y = detected_contours[i][j].y;

      temp.end.x = detected_contours[i][j+1].x;
      temp.end.y = detected_contours[i][j+1].y;

      edges.push_back(temp);
      set.push_back(temp);
    }
    
    Edge temp;
    temp.start.x = detected_contours[i][detected_contours[i].size()-1].x;
    temp.start.y = detected_contours[i][detected_contours[i].size()-1].y;

    temp.end.x = detected_contours[i][0].x;
    temp.end.y = detected_contours[i][0].y;

    edges.push_back(temp);
    set.push_back(temp);

    edge_sets.push_back(set);
  }
  //ROS_INFO("Edges.size(): %i", (int)edges.size());





  /*ros::Time t_start_cirs_from_edges = ros::Time::now();
  
  cv::Point robo_cen;
  robo_cen.x = 0;
  robo_cen.y = 0;

  std::vector<Circle> cirs_from_edges = getCirclesFromEdges(edges, robo_cen);
  ros::Duration d_cirs_from_edges(ros::Time::now() - t_start_cirs_from_edges);

  //ROS_INFO("cirs_from_edges size: %i", (int)cirs_from_edges.size());
  for(int i=0;i<cirs_from_edges.size();i++)
  {
    ROS_INFO("Circle %i - Center: (%i, %i) Radius: %f", i, cirs_from_edges[i].center.x, cirs_from_edges[i].center.y, cirs_from_edges[i].radius);
  }*/

  ros::Time t_start_cirs_from_sets = ros::Time::now();
  std::vector<Circle> cirs_from_sets = getCirclesFromEdgeSets(edge_sets);
  ros::Duration d_cirs_from_sets(ros::Time::now() - t_start_cirs_from_sets);

  for(int i=0;i<cirs_from_sets.size();i++)
  {
    //ROS_INFO("Circle %i - Center: (%f, %f) Radius: %f", i, cirs_from_sets[i].center.x, cirs_from_sets[i].center.y, cirs_from_sets[i].radius);
    result.push_back(cirs_from_sets[i]);
  }


  /*
   * Get every set of edges, and make a circle for each set
   */


  // Find convex hull for each set of contour points
  /*std::vector< std::vector<cv::Point> > hull(detected_contours.size());
  for(int i=0;i<detected_contours.size();i++)
  {
    cv::convexHull( detected_contours[i], hull[i], false );
  }
  
  ROS_INFO("hull.size(): %i", (int)hull.size());
  
  // Build a polygon for each convex hull
  for(int h=0;h<hull.size();h++)
  {
    Polygon p;
    for(int i=0;i<hull[h].size()-1;i++)
    {
      std::cout<<"\nPoint "<<i<<" ("<<hull[h][i].x<<", "<<hull[h][i].y<<")";
      Edge temp;
      temp.start.x = hull[h][i].x;
      temp.start.y = hull[h][i].y;

      temp.end.x = hull[h][i+1].x;
      temp.end.y = hull[h][i+1].y;

      p.edges.push_back(temp);
    }
    
    // Last edge
    Edge temp;
    temp.start.x = hull[h][hull[h].size()-1].x;
    temp.start.y = hull[h][hull[h].size()-1].y;

    temp.end.x = hull[h][0].x;
    temp.end.y = hull[h][0].y;

    p.edges.push_back(temp);
   
    // Build the set of normals for the polygon 
    for(int i=0;i<p.edges.size();i++)
    {
      p.normals.push_back(computeNormal(p.edges[i]));
    }
    
    // Pack the polygon and return the set of circles 
    std::vector<Circle> cirs = getCirclesFromPoly(p);
    ROS_INFO("cirs.size(): %i", (int)cirs.size());

    result.push_back(cirs);
  }
  
  ROS_INFO("result size: %i", (int)result.size());
  for(int i=0;i<result.size();i++)
  {
    ROS_INFO("result[%i].size(): %i", i, (int)result[i].size());
    for(int j=0;j<result[i].size();j++)
    {
      ROS_INFO("Circle %i, Center: (%i, %i) Radius: %f", j, result[i][j].center.x, result[i][j].center.y, result[i][j].radius);
    }
  }*/

  //ROS_INFO("d_edges_detect: %f", d_edges_detect.toSec());
  //ROS_INFO("d_contour: %f", d_contour.toSec());
  //ROS_INFO("d_cirs_from_edges: %f", d_cirs_from_edges.toSec());
  //ROS_INFO("d_cirs_from_sets: %f", d_cirs_from_sets.toSec());

  //ROS_INFO("Leaving go()");

  return result;
}
