#ifndef CIRCLE_PACKER
#define CIRCLE_PACKER
#include "utility.h"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <queue>

struct Edge
{
  cv::Point start;
  cv::Point end;
};

struct Normal
{
  double a, b, c;
};

struct Triangle
{
  std::vector<Edge> edges;
};


struct Polygon
{
  std::vector<Edge> edges;
  std::vector<Normal> normals;
};

struct Cell
{
  cv::Point p;
  double dist;
};


struct Circle
{
  cv::Point center;
  double radius;
};

struct CompareDist
{
  bool operator()(const Cell& c1, const Cell& c2)
  {
    return c1.dist < c2.dist;
  }
};


class CirclePacker 
{
  public:
    CirclePacker(cv::Mat);
    ~CirclePacker();

    void CannyThreshold(int, void*);
    double getMinDistToPoly(const Polygon&, const Cell&);
    double getMinDistToCirs(const std::vector<Circle>&, const Cell&);
    void deleteCellsInCir(const std::vector<Cell>&, const Circle, std::vector<Cell>&);

    Normal computeNormal(Edge);
    bool cellInPoly(Polygon, cv::Point);
    std::vector<Circle> getCirclesFromPoly(Polygon);
    std::vector<Triangle> triangulatePolygon(const Polygon&);
    std::vector<Cell> getCellsFromEdges(const std::vector<Edge>);

    std::vector<Circle> go();
  private:
    cv::Mat src, src_gray;
    cv::Mat dst, detected_edges;

    int edgeThresh = 1;
    int lowThreshold;
    int const max_lowThreshold = 100;
    int ratio = 3;
    int kernel_size = 3;
    std::string window_name = "Edge Map";
    //cv::RNG rng(12345);

};

#endif
