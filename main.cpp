#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

void apply_edge_detection(Mat image) {

    Mat gray_image;
    Mat blur_image;
    Mat threshold_image;

    Size kernal_size = Size(9,9);
    
    // Grayscale image
    cvtColor(image, gray_image, cv::COLOR_RGB2GRAY );
    
    // Apply blur to image
    GaussianBlur(gray_image, blur_image, kernal_size, 0);

    imshow("Blur image", blur_image);

    waitKey(0);

    // Threshold image with otsu thresholding
    threshold(blur_image, threshold_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    //  morphologyEx(threshold_image, threshold_image, cv::MORPH_GRADIENT, kernel_size)
    imshow("Threshold image", threshold_image);
}

void rotate_image(Mat image, double angle) {
    // Get rotation matrix
    Point2f center(image.cols / 2.0, image.rows / 2.0);
    Mat rotationMatrix = getRotationMatrix2D(center, angle, 1.0);

    // Find new image size
    double radians = angle * CV_PI / 180.0;
    double sinAngle = std::abs(sin(radians));
    double cosAngle = std::abs(cos(radians));
    
    // determine bounding rectangle, center not relevant
    RotatedRect rotatedDimension = RotatedRect(Point2d(), image.size(), angle);
    
    Rect2f thing = rotatedDimension.boundingRect2f();
    rectangle(image, thing, 1);
    // std::cout << rotatedDimension.height << end;

    int newWidth = static_cast<int>(image.cols * cosAngle + image.rows * sinAngle);
    int newHeight = static_cast<int>(image.cols * sinAngle + image.rows * cosAngle);
    Size newImageSize = Size(newWidth,newHeight);

    // Adjust rotation matrix by new center
    
    rotationMatrix.at<double>(0, 2) += newWidth/2.0 - image.cols/2.0;
    rotationMatrix.at<double>(1, 2) += newHeight/2.0 - image.rows/2.0;

    // Rotate the image
    Mat rotatedImage;
    warpAffine(image, rotatedImage, rotationMatrix, newImageSize);

    // Display the original and rotated images
    imshow("Original Image", image);
    imshow("Rotated Image", rotatedImage);
    waitKey(0);
}

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

    String windowName = "Original Image"; //Name of the window

    namedWindow(windowName); // Create a window
    rotate_image(image, 10.0);
    apply_edge_detection(image);
    imshow(windowName, image); // Show our image inside the created window.

    waitKey(0); // Wait for any keystroke in the window

    destroyWindow(windowName); //destroy the created window

    return 0;
}

