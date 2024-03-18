#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

struct LineCounter {
  float gradient;
  float constant;
  int pixel_count;
};

// vector<LineCounter> count_lines(vector<Vec4i> lines) {

//     vector<LineCounter> lineCount;
//     LineCounter 
//     if (lines.size() < 1)
//   // Iterate over lines to find most likely candidate for horizon
//   for (size_t i = 0; i < lines.size(); i++) {
//     line(gray_image, Point(lines[i][0], lines[i][1]),
//          Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
//   }
//     return vector<LineCounter>
// } 

/**
 * @brief Use Otsu thresholding to seperate foreground/background to find horizon. 
 * Then use edge detection and line detection to find the horizon. 
 * Otsu was chosen since it can account for more colours of the sky as well as ignore the edges that would exist in the ocean.
 * 
 * @param image Image to detect horizon in
 * @param showImage Show the intermediate image steps
 * @return double Angle of horizon detected in degrees
 */
pair<double,double> find_horizon(Mat image, bool showImage) {

    Mat gray_image;
    Mat blur_image;
    Mat threshold_image;
    Mat edge_image;
    
    // Grayscale image
    cvtColor(image, gray_image, cv::COLOR_RGB2GRAY );
    
    // Apply blur to image
    medianBlur(gray_image, blur_image, 15);

    if (showImage) {
        imshow("Blurred Grayscale Image", blur_image);
        waitKey(0);
    }

    // Threshold image with otsu thresholding
    threshold(blur_image, threshold_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Show thresholded image
    if (showImage) {
      imshow("Thresholded Image", threshold_image);
      waitKey(0);
    }

    // Perform edge detection
    Canny(threshold_image, edge_image, 20, 80, 3);

    if (showImage) {
      imshow("Edge Image", edge_image);
      waitKey(0);
    }

    // Find straight lines in image using Probabilistic Hough Lines
    vector<Vec4i> lines;
    HoughLinesP(edge_image, lines, 5, CV_PI / 180, 80, 30, 1);


    // Iterate over lines to find most likely candidate for horizon
    for (size_t i = 0; i < lines.size(); i++) {
      line(gray_image, Point(lines[i][0], lines[i][1]),
           Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
    }

    if (showImage) {
      imshow("Thresholded Image", gray_image);
      waitKey(0);
    }
    
    pair<double, double> results = pair(-1.0, 85);
    return results;
}

/**
 * @brief Rotate image and crop to get rid of blank pixels. Assumes image is w > h for simplicity
 * 
 * @param image Image to rotate
 * @param angle Angle in degrees. Follows right hand rule
 * @param showImage Show intermediate steps for rotation
 * @return Mat Rotated and cropped image.
 */
Mat rotate_image(Mat image, double angle, bool showImage) {
    
    // Find new image size
    Point2f center(image.cols / 2.0, image.rows / 2.0);
    
    // Get rotation matrix
    Mat rotationMatrix = getRotationMatrix2D(center, angle, 1.0);

    // Find size of new uncropped image
    double radians = angle * CV_PI / 180.0;
    double sinAngle = std::abs(sin(radians));
    double cosAngle = std::abs(cos(radians));

    int newWidth = static_cast<int>(image.cols * cosAngle + image.rows * sinAngle);
    int newHeight = static_cast<int>(image.cols * sinAngle + image.rows * cosAngle);
    Size newImageSize = Size(newWidth,newHeight);

    // Adjust rotation matrix by new center
    rotationMatrix.at<double>(0, 2) += newWidth/2.0 - image.cols/2.0;
    rotationMatrix.at<double>(1, 2) += newHeight/2.0 - image.rows/2.0;

    // Rotate the image matrix
    Mat rotatedImage;
    warpAffine(image, rotatedImage, rotationMatrix, newImageSize);
    
    // Display the rotated image
    if (showImage) {
        imshow("Rotated Image", rotatedImage);
        waitKey(0);
    }

    // Determine bounding rectangle, center not relevant
    Point2f vertices[4];
    Point2f rotatedCenter(newImageSize.width / 2.0, newImageSize.height / 2.0);
    
    RotatedRect rotatedDimension = RotatedRect(rotatedCenter, image.size(), angle);
    rotatedDimension.points(vertices);

    // Assume w > h image. Find cropped dimensions
    int croppedLowerX, croppedLowerY, croppedUpperX, croppedUpperY;

    if (angle > 0) {
      croppedLowerX = min(vertices[1].x, vertices[2].x);
      croppedLowerY = max(vertices[1].y, vertices[2].y);
      croppedUpperX = max(vertices[0].x, vertices[3].x);
      croppedUpperY = min(vertices[0].y, vertices[3].y);
    } else {
      croppedUpperX = min(vertices[2].x, vertices[3].x);
      croppedUpperY = max(vertices[2].y, vertices[3].y);
      croppedLowerX = max(vertices[0].x, vertices[1].x);
      croppedLowerY = min(vertices[0].y, vertices[1].y);
    }

    // Crop image to new dimensions
    Mat croppedImage = rotatedImage(Range(croppedLowerY, croppedUpperY),
                                    Range(croppedLowerX, croppedUpperX));

    // Display the cropped rotated image
    if (showImage) {
        imshow("Cropped Rotated Image", croppedImage);
        waitKey(0);
    }

    return croppedImage;

}

void detect_ships(Mat image, int horizon_height) {
    Mat cropped_image;
    Mat gray_image;
    Mat threshold_image;
    Mat edge_image;

    Size kernal_size = Size(3,3);

    // Crop image to above horizon
    cropped_image = image( Range(0, horizon_height), Range(0, image.cols - 1));
    imshow("Cropped Image", cropped_image);

    // Grayscale image
    cvtColor(cropped_image, gray_image, cv::COLOR_RGB2GRAY );
    adaptiveThreshold(gray_image, threshold_image, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                      cv::THRESH_BINARY, 15, 10);
    // // Apply blur to image
    // GaussianBlur(gray_image, blur_image, kernal_size, 0);
    imshow("Thresholded Image", threshold_image);

    Canny(threshold_image, edge_image, 10, 100, 3);
    Mat kernel = getStructuringElement(1, Size(3,3));
    morphologyEx(edge_image, edge_image, cv::MORPH_CLOSE, kernel);

    imshow("Edge Image", edge_image);
    waitKey(0);

    vector<vector<Point>> contours;
    findContours(edge_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, cropped_image.rows - horizon_height ));
    drawContours(image, contours, -1, Scalar(0, 0, 255), 1);

    for (size_t i = 0; i < contours.size(); i++) {
      Rect boundingBox = cv::boundingRect(contours[i]);
      rectangle(image, boundingBox, Scalar(0,255,0), 1);
    } 

    imshow("Edge Image", image);
    waitKey(0);
}

bool showHorizonImages = false;
bool showRotateImages = false;

int main(int argc, char** argv)
{
    // Read the image file
    Mat image = imread("three_ships_horizon.png");

    // Check for failure
    if (image.empty()) 
    {
        cout << "Could not open or find the image" << endl;
        cin.get(); //wait for any key press
        return -1;
    }

    imshow("Original Image", image);
    waitKey(0);

    pair<double, double> horizon_results = find_horizon(image, showHorizonImages);

    Mat rotatedImage = rotate_image(image, horizon_results.first, showRotateImages);
    // imshow("Rotated Image", rotatedImage);
    waitKey(0);

    detect_ships(rotatedImage, 75);

    return 0;
}

