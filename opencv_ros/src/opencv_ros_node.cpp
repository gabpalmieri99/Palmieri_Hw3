#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

static const std::string OPENCV_WINDOW = "Image window";

class OpenCVNode
{
private:
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  cv::Mat image_with_contours_;

public:
  OpenCVNode() : it_(nh_)
  {
    // Inizializzazione delle sottoscrizioni e delle pubblicazioni ROS
    image_sub_ = it_.subscribe("/iiwa/camera1/image_raw", 1, &OpenCVNode::imageCb, this);
    image_pub_ = it_.advertise("/opencv_node/output_image", 1);

    // Creazione della finestra di visualizzazione OpenCV
    cv::namedWindow(OPENCV_WINDOW);
  }

  ~OpenCVNode()
  {
    // Chiusura della finestra di visualizzazione OpenCV
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr &msg);

  void detectBlobs(cv::Mat &image);

};

void OpenCVNode::imageCb(const sensor_msgs::ImageConstPtr &msg)
{
  // Conversione dell'immagine ROS in formato OpenCV
  cv_bridge::CvImagePtr cv_ptr;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Detect blobs using OpenCV SimpleBlobDetector
  detectBlobs(cv_ptr->image);

  // Update GUI Window
  cv::imshow(OPENCV_WINDOW, cv_ptr->image);
  cv::waitKey(1);

  // Output modified video stream
   image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_with_contours_).toImageMsg());

}

void OpenCVNode::detectBlobs(cv::Mat &image)
{
cv::SimpleBlobDetector::Params params;
params.filterByColor = true;
params.blobColor = 255;  // Valore del colore da rilevare (255 per bianco)
params.filterByArea = false;
params.filterByCircularity = false;
params.filterByConvexity = false;
params.filterByInertia = false;
cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

// Converti l'immagine in spazio colore HSV
cv::Mat hsv_image;
cv::cvtColor(image, hsv_image, cv::COLOR_BGR2HSV);

// Definisci i range di colore per il rosso in HSV
cv::Scalar lower_red = cv::Scalar(0, 100, 100);  // Valori minimi di HSV per il rosso
cv::Scalar upper_red = cv::Scalar(10, 255, 255);  // Valori massimi di HSV per il rosso

// Applica una maschera per isolare il rosso
cv::Mat red_mask;
cv::inRange(hsv_image, lower_red, upper_red, red_mask);

// Detect blobs nell'immagine mascherata
std::vector<cv::KeyPoint> keypoints;
detector->detect(red_mask, keypoints);

// Draw detected blobs as green circles
cv::drawKeypoints(image, keypoints, image_with_contours_, cv::Scalar(0, 255, 0), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
// Show blobs
cv::imshow("keypoints", image_with_contours_);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_node");
  OpenCVNode opencv_node;

  ros::spin();
  return 0;
}
