#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

/**
 * @brief Find the horizon based on the average of the lines provided. Find gradient based on left and right most lines.
 * Assumes no outlier detections. Assume edges of images are unobstructed by other flat objects.
 * Outlier detection can be implemented by comparing to existing horizon solutions.
 * 
 * @param lines Vector of 4 integers representing line coordinates (x1,y1,x2,y2)
 * @return pair<double,double> Angle in degrees, horizon y
 */
pair<double,double> findBestFitLine(const vector<Vec4i> &lines);

/**
 * @brief Use Otsu thresholding to seperate foreground/background and then perform Canny edge detection to find horizon. 
 * Use Probabilistic Hough Lines detection to find sufficiently long enough straight lines in the edges to find horizon edge candidates. 
 * Otsu was chosen since it can account for more colours of the sky as well as ignore the edges that would exist in the ocean.
 * 
 * @param image Image to detect horizon in
 * @param showImage Show the intermediate image steps
 * @return double Angle of horizon detected in degrees
 */
pair<double,double> find_horizon(Mat &image, bool showImage);

/**
 * @brief Rotate image by multipling by rotation matrix and crop to get rid of blank pixels.
 * Assumes image is w > h for simplicity
 * 
 * @param image Image to rotate
 * @param angle Angle in degrees. Follows right hand rule
 * @param showImage Show intermediate steps for rotation
 * @return Mat Rotated and cropped image.
 */
Mat rotate_image(Mat &image, double angle, bool showImage);


/**
 * @brief Detect ships and draw bounding boxes on the image 
 * Detect ships in the image by thresholding -> edge detection -> closing morphological -> finding contours -> filter contours to find ships. 
 * Use adaptive gaussian mean thresholding to generally detect ships since the colours of the ships are close to the ocean colour.
 * Edges are detected and use closing morphological detection to close some contours. Find the contours and filter small contours and ones below horizon.
 * 
 * If many detections were on the same ship, non-maximal suppression can be implemented to remove overlapping bounding boxes. This wasn't required in this case.
 * 
 * Assumes ships are far away on the horizon by ignoring objects below horizon. Choose to use a different method if ships are close.
 * An option would be another thresholding below horizon with bias towards larger objects. 
 * 
 * @param image Image to rotate
 * @param horizon_height. Horizon pixel height in image
 * @param showImage Show intermediate steps
 * @return
 */
void detect_ships(Mat &image, int horizon_height, bool showImage);