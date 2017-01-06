#include "circle_packer.h"



CirclePacker::CirclePacker(cv::Mat s)
{
  src = s;
}

void CirclePacker::CannyThreshold(int, void*)
{
  /// Reduce noise with a kernel 3x3
  blur( src_gray, detected_edges, cv::Size(3,3) );

  /// Canny detector
  Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

  /// Using Canny's output as a mask, we display our result
  dst = cv::Scalar::all(0);

  src.copyTo( dst, detected_edges);
  imshow( window_name, dst );
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
  std::cout<<"\ne.start: "<<e.start.x<<" "<<e.start.y;
  std::cout<<"\ne.end: "<<e.end.x<<" "<<e.end.y;
  Normal result;
  result.a = e.end.y - e.start.y;
  result.b = -(e.end.x - e.start.x);

  result.c = -((result.a*e.start.x) + (result.b*e.start.y));

  std::cout<<"\na: "<<result.a<<" b: "<<result.b<<" c: "<<result.c;
  return result;
}


bool CirclePacker::cellInPoly(Polygon poly, cv::Point cell)
{
  for(int i=0;i<poly.normals.size();i++)
  {
    std::cout<<"\nnormal a: "<<poly.normals[i].a<<" b: "<<poly.normals[i].b<<" c: "<<poly.normals[i].c;
    double d = poly.normals[i].a*cell.x + poly.normals[i].b*cell.y + poly.normals[i].c;
    std::cout<<"\ncell center: "<<cell.x<<", "<<cell.y<<" d: "<<d;
    if(d > -0.000001)
    {
      std::cout<<"\nNot in polygon";
      return false;
    }
  }

  return true;
}


std::vector<Circle> CirclePacker::getCirclesFromPoly(Polygon poly)
{
  std::cout<<"\n# of edges: "<<poly.edges.size();
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

  std::cout<<"\nMAX_WIDTH: "<<MAX_WIDTH<<" MAX_LENGTH: "<<MAX_LENGTH<<" width_count: "<<width_count<<" length_count: "<<length_count;

  std::vector<Cell> cells;
 
  for(int i=0;i<width_count;i++)
  {
    for(int j=0;j<length_count;j++)
    {
      std::cout<<"\ni: "<<i<<" j: "<<j<<" round: "<<round;
      double x = start_x + (round * (i)); 
      double y = start_y + (round * (j));
      Cell temp;
      temp.p.x = x;
      temp.p.y = y;
    
      std::cout<<"\n("<<temp.p.x<<", "<<temp.p.y<<")";

      if(cellInPoly(poly, temp.p))
      {
        cells.push_back(temp);
      }
    }
  }
  

  std::vector<Cell> reduced_cells = cells;


  while(cells.size() > 0)
  {
    std::cout<<"\nIn while cells.size(): "<<cells.size()<<" result.size(): "<<result.size();
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

      temp.center = c.p;
      temp.radius = c.dist;

      result.push_back(temp);
    }
  }
  
  std::cout<<"\nFinal number of circles: "<<result.size();
  for(int i=0;i<result.size();i++)
  {
    std::cout<<"\nCircle "<<i<<" ("<<result[i].center.x<<", "<<result[i].center.y<<") radius: "<<result[i].radius;
  }

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


std::vector<Cell> CirclePacker::getCellsFromEdges(const std::vector<Edge> edges)
{
  std::vector<Cell> result;

  return result;
}


std::vector<Circle> CirclePacker::go()
{
  // Create a matrix of the same size and type as src
  dst.create( src.size(), src.type() );

  // Convert to grayscale
  cvtColor(src, src_gray, CV_BGR2GRAY);

  // Get the edges
  CannyThreshold(0, 0);

  // Get the contour points
  std::vector< std::vector<cv::Point> > detected_contours;
  std::vector<cv::Vec4i> hierarchy;
  cv::findContours(detected_edges, detected_contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

  // Find convex hull for each set of contour points
  std::vector< std::vector<cv::Point> > hull(detected_contours.size());
  for(int i=0;i<detected_contours.size();i++)
  {
    cv::convexHull( detected_contours[i], hull[i], false );
  }
  
  
  // Build a polygon for each convex hull
  Polygon p;
  for(int i=0;i<hull[2].size()-1;i++)
  {
    std::cout<<"\nPoint "<<i<<" ("<<hull[2][i].x<<", "<<hull[2][i].y<<")";
    Edge temp;
    temp.start.x = hull[2][i].x;
    temp.start.y = hull[2][i].y;

    temp.end.x = hull[2][i+1].x;
    temp.end.y = hull[2][i+1].y;

    p.edges.push_back(temp);
  }
  
  // Last edge
  Edge temp;
  temp.start.x = hull[2][hull[2].size()-1].x;
  temp.start.y = hull[2][hull[2].size()-1].y;

  temp.end.x = hull[2][0].x;
  temp.end.y = hull[2][0].y;

  p.edges.push_back(temp);
  
  // Build the set of normals for the polygon 
  for(int i=0;i<p.edges.size();i++)
  {
    p.normals.push_back(computeNormal(p.edges[i]));
  }
  
  // Pack the polygon and return the set of circles 
  std::vector<Circle> cirs = getCirclesFromPoly(p);
}
