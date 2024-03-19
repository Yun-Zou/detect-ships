#include "main.hpp"

pair<double,double> findBestFitLine(const vector<Vec4i> &lines) {

    double sumIntercept = 0.0;
    double sumSlope = 0.0;
    double leftMostX = lines[0][2];
    double leftMostY;
    double rightMostX = 0.0;
    double rightMostY;

    for (auto line : lines) {
      sumIntercept += ((line[1] + line[3]) / 2.0);

      if (line[3] < leftMostX) {
        leftMostX = line[0];
        leftMostY = line[1];
      }
      if (line[2] > rightMostX) {
        rightMostX = line[2];
        rightMostY = line[3];
      }
    }   

    double averageSlope = 180 / CV_PI * atan((rightMostY - leftMostY) / (rightMostX - leftMostX));
    double averageIntercept = sumIntercept / lines.size();

    pair<double,double> result = make_pair(averageSlope, averageIntercept);
    return result;
}

pair<double,double> findHorizon(Mat &image, bool showImage) {

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
    HoughLinesP(edge_image, lines, 5, CV_PI / 180, 80, 30, 2);


    // Iterate over lines to find most likely candidate for horizon
    for (size_t i = 0; i < lines.size(); i++) {
      line(gray_image, Point(lines[i][0], lines[i][1]),
           Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
    }

    if (showImage) {
      imshow("Thresholded Image", gray_image);
      waitKey(0);
    }
    
    // Fit a line to the lines we detected
    pair<double,double> results = findBestFitLine(lines);
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
Mat rotateImage(Mat &image, double angle, bool showImage) {
    
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


void detectShips(Mat &image, int horizon_height, bool showImage) {
    Mat cropped_image;
    Mat gray_image;
    Mat threshold_image;
    Mat edge_image;

    // Crop image to above horizon
    cropped_image = image( Range(0, horizon_height + 3), Range(0, image.cols - 1));

    if (showImage) {
      imshow("Cropped Image", cropped_image);
      waitKey(0);
    }

    // Grayscale image
    cvtColor(cropped_image, gray_image, cv::COLOR_RGB2GRAY );

    // Apply adative thresholding
    adaptiveThreshold(gray_image, threshold_image, 255, cv::ADAPTIVE_THRESH_MEAN_C,
                      cv::THRESH_BINARY, 15, 10);

    // Apply blur to image
    // Size kernal_size = Size(3,3);
    // GaussianBlur(threshold_image, threshold_image, kernal_size, 0);

    if (showImage) {
      imshow("Thresholded Image", threshold_image);
      waitKey(0);
    }

    // Apply Canny Thresholding to Image
    Canny(threshold_image, edge_image, 5, 40, 3);

    // Apply Closing Morphological Operation to remove small holes
    Mat kernel = getStructuringElement(1, Size(3,3));
    morphologyEx(edge_image, edge_image, cv::MORPH_CLOSE, kernel);

    if (showImage) {
      imshow("Edge Image", edge_image);
      waitKey(0);
    }

    vector<vector<Point>> contours;
    findContours(edge_image, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, Point(0, cropped_image.rows - horizon_height - 3 ));
    // drawContours(image, contours, -1, Scalar(0, 0, 255), 1);

    // Iterate over contours to draw boxes
    for (size_t i = 0; i < contours.size(); i++) {

      // Remove contour areas which are too small
      if (contourArea(contours[i]) < 30) {
        continue;
      }

      // Output bounding boxes if they're not impossibly close to ocean
      // If too many overlapping detections are found, implement non-maximal suppresion
      if (contours[i][0].y < horizon_height - 5) {
        Rect boundingBox = cv::boundingRect(contours[i]);
        rectangle(image, boundingBox, Scalar(0,255,0), 1);
      }
    } 

}

bool showHorizonImages = false;
bool showRotateImages = false;
bool showShipImages = false;

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

    // Find angle and height of horizon
    pair<double, double> horizon_results = findHorizon(image, showHorizonImages);

    // Rotate image and crop
    Mat rotatedImage = rotateImage(image, horizon_results.first, showRotateImages);

    // Detect ships and place bounding boxes
    detectShips(rotatedImage, horizon_results.second, showShipImages);
    imshow("Final Image", rotatedImage);
    waitKey(0);

    // Write image to folder
    imwrite("./three_ships_boxed.TIFF", rotatedImage);
    return 0;
}

