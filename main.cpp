#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

double find_horizon(Mat image, bool showImage) {

    Mat gray_image;
    Mat blur_image;
    Mat threshold_image;

    Size kernal_size = Size(15,15);
    
    // Grayscale image
    cvtColor(image, gray_image, cv::COLOR_RGB2GRAY );
    
    // Apply blur to image
    GaussianBlur(gray_image, blur_image, kernal_size, 0);

    if (showImage) {
        imshow("Blurred Grayscale Image", blur_image);
        waitKey(0);
    }

    // Threshold image with otsu thresholding
    threshold(blur_image, threshold_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    // Canny(blur_image, threshold_image, 20, 80, 3);
    // morphologyEx(threshold_image, edge_image, cv::MORPH_GRADIENT,)
    if (showImage) {
        imshow("Thresholded Image", threshold_image);
        waitKey(0);
    }

    //  morphologyEx(threshold_image, threshold_image, cv::MORPH_GRADIENT, kernel_size)
    // imshow("Threshold image", threshold_image);

    return 5.0;
}

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

    // Find cropped dimensions
    int croppedUpperX, croppedUpperY, croppedLowerX, croppedLowerY;

   // Draw the rotated rectangle
    for (int i = 0; i < 4; i++) {
        
        std::cout << vertices[i].x << " " << vertices[i].y << endl;
    }
    // Size2f uprightSize(boundingBox.width, boundingBox.height);
    // std::cout << boundingBox.width << " " << boundingBox.height << endl;
    // rectangle(rotatedImage, boundingBox, cv::Scalar(0,255,0));
    
    // Crop image to new dimensions
    Mat croppedImage = rotatedImage( Range(105, 446), Range(79, 598));

    // Display the cropped rotated image
    if (showImage) {
        imshow("Cropped Rotated Image", croppedImage);
        waitKey(0);
    }

    return croppedImage;

}

void detect_ships(Mat image) {
    Mat cropped_image;
    Mat gray_image;
    Mat blur_image;
    Mat edge_image;

    Size kernal_size = Size(5,5);

    // Crop image to above horizon
    cropped_image = image( Range(0, 85), Range(0, image.cols - 1));
    // imshow("Cropped Image", cropped_image);
    
    // Grayscale image
    cvtColor(cropped_image, gray_image, cv::COLOR_RGB2GRAY );
    
    // Apply blur to image
    GaussianBlur(gray_image, blur_image, kernal_size, 0);

    imshow("Blurred Image", blur_image);

    Canny(blur_image, edge_image, 10, 100, 3);

    imshow("Thresholded Image", edge_image);
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


    double angle = find_horizon(image, showHorizonImages);

    // Mat rotatedImage = rotate_image(image, angle, showRotateImages);

    detect_ships(image);

    return 0;
}

