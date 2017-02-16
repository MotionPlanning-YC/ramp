#ifndef BLOB_DETECTOR_H
#define BLOB_DETECTOR_H
#include "opencv2/features2d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
using namespace cv;


typedef void (*MomentsInTileFunc)(const Mat& img, double* moments);


struct Center
{
  cv::Point2d location;
    double radius;
    double confidence;
};

class BlobDetector : public SimpleBlobDetector
{
public:
  BlobDetector();
  ~BlobDetector();

  cv::Mat src;

  void detect(std::vector<cv::KeyPoint>& keypoints);
  void findBlobs(Mat binaryImage, std::vector<Center>& centers);
  Moments buildMoment(Mat _src, bool binary);

  Params params;
};

#endif
