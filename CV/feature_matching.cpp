
#include <iostream>
#include <vector>
#include <fstream>
#include "opencv2/opencv.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/core.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
using namespace cv;
using namespace cv::xfeatures2d;
using std::cout;
using std::endl;
using std::vector;
using std::ifstream;
using std::string;
const char* keys =
    "{ help h |                  | Print help message. }"
    "{ input1 | box.png          | Path to input image 1. }"
    "{ input2 | box_in_scene.png | Path to input image 2. }"
    "{ input3 | thresholds.txt   | Path to threshold file. }";

Mat cubeDetector(Mat img, vector<int> thresh, Scalar color) {
      Mat hsv = Mat(img.size(), img.type());
    cvtColor(img, hsv, COLOR_BGR2HSV);

    //cv2.imwrite("flowchart_stuff/hsv.jpg", hsv)

    vector<Mat> channels;
    split(hsv, channels);
    // 0 = H, 1 = S, 2 = V

    Mat h;
    resize(channels[0], h, Size(480,640));
    Mat s;
    resize(channels[1], s, Size(480,640));
    Mat v;
    resize(channels[2], v, Size(480,640));
    imshow("H", h);
    imshow("S", s);
    imshow("V", v);

    inRange(channels[0], thresh[0], thresh[1], channels[0]);
    inRange(channels[1], thresh[2], thresh[3], channels[1]);
    inRange(channels[2], thresh[4], thresh[5], channels[2]);

    //cv2.imshow("goldH", cv2.resize(channels[0], (320, 480)))
    //cv2.imshow("goldS", cv2.resize(channels[1], (320, 480)))
    //cv2.imshow("goldV", cv2.resize(channels[2], (320, 480)))

    Mat finalMap = Mat(img.size(), img.type());
    bitwise_and(channels[0], channels[1], finalMap);
    bitwise_and(finalMap, channels[2], finalMap);

    //cv2.imshow("combined", cv2.resize(finalMap, (480, 640)))

    //cv2.imwrite("flowchart_stuff/goldBinary.jpg", finalMap)

    //floodFillMap = pyCopy.deepcopy(finalMap)

    //cv2.floodFill(floodFillMap,None,(0,0),255)

    //floodFillMap = cv2.bitwise_not(floodFillMap)

    //finalMap = cv2.bitwise_or(finalMap, floodFillMap)

    //finalMap = cv2.GaussianBlur( finalMap, (9, 9), 2, 2)

    //cv2.imwrite("flowchart_stuff/goldBinaryBlur.jpg", finalMap)

    //cv2.imshow("cubeBinaryMap", cv2.resize(finalMap, (480, 640)))


    //vector<vector<Point>>  finalMapContours;
    //findContours(finalMap  ,finalMapContours, RETR_TREE, CHAIN_APPROX_SIMPLE);

    //if drawnImg is not None:
        //retImg = pyCopy.deepcopy(drawnImg)
    //else:
       //retImg = pyCopy.deepcopy(img)

    Mat retImg = img.clone();

    // if(finalMapContours.size() != 0) {
    //    vector<vector<Point>> polygons;
    //    for(size_t i = 0; i < finalMapContours.size(); i++) {
    //       vector<Point> contour = finalMapContours[i];
    //        if(contourArea(contour) > 100000) {
    //            vector<Point> polygon;
    //            approxPolyDP(contour, polygon, .05 * arcLength(contour, true), true);
    //            polygons.push_back(polygon);
    //            Moments M = moments(contour);
    //            int cX = int(M.m10 / M.m00);
    //            int cY = int(M.m01 / M.m00);
    //
    //            // draw the contour and center of the shape on the image
    //            circle(retImg, Point(cX, cY), 50,  Scalar(255,255,255) , -1);
    //          }
    //
    //     drawContours(retImg, polygons, -1, color, 20);
    //     }
    //   }
    // img
    return finalMap;
}

int main( int argc, char* argv[] )
{
    CommandLineParser parser( argc, argv, keys );
    Mat img1 = imread(argv[1],  IMREAD_GRAYSCALE );
    Mat img2 = imread(argv[2],  IMREAD_GRAYSCALE );
    Mat img3 = imread(argv[2]);
    if ( img1.empty() || img2.empty() )
    {
        cout << "Could not open or find the image!\n" << endl;
        parser.printMessage();
        return -1;
    }

    ifstream thresh_file;
    thresh_file.open(argv[3]);
    if (!thresh_file) {
      std::cerr << "Unable to open file datafile.txt";
      exit(1);   // call system to stop
    }

    vector<int> orange_thresh;
    vector<int> purple_thresh;
    vector<int> green_thresh;
    string s;
    int j = 0;
    while(thresh_file >> s) {
      if(j == 0) {
        if(s.compare(";") == 0) {
          j = 1;
        }
        else {
          orange_thresh.push_back(atoi(s.c_str()));
        }
      }
      else if(j == 1) {
        if(s.compare(";") == 0) {
          j = 2;
        }
        else {
          purple_thresh.push_back(atoi(s.c_str()));
        }
      }
      else {
        if(s.compare(";") != 0) {
          green_thresh.push_back(atoi(s.c_str()));
        }
      }
    }


    Mat orange_mat = cubeDetector(img3, orange_thresh, Scalar(0,0, 255));
    Mat purple_mat = cubeDetector(img3, purple_thresh, Scalar(255,0, 0));
    Mat green_mat = cubeDetector(img3, green_thresh, Scalar(0, 255, 0));

    Mat ret_img = green_mat;
    //Mat ret_img = Mat(orange_mat.size(), orange_mat.type());
    //bitwise_or(orange_mat, purple_mat, ret_img);
    //bitwise_or(ret_img, green_mat, ret_img);


    Mat ret_img_show = Mat(Size(480, 640), ret_img.type());
    resize(ret_img, ret_img_show, Size(480, 640));
    imshow("cube_detector", ret_img_show);

    Mat img2_masked = Mat(Size(480, 640), img2.type());
    img2.copyTo(img2_masked, ret_img);
    Mat img2_show = Mat(Size(480, 640), img2.type());
    resize(img2_masked, img2_show, Size(480, 640));
    imshow("img2", img2_show);



    //-- Step 1: Detect the keypoints using SURF Detector, compute the descriptors
    int minHessian = 800;
    Ptr<SURF> detector = SURF::create( minHessian );
    vector<KeyPoint> keypoints1, keypoints2;
    Mat descriptors1, descriptors2;
    detector->detectAndCompute( img1, noArray(), keypoints1, descriptors1 );
    detector->detectAndCompute( img2_masked, noArray(), keypoints2, descriptors2 );
    //-- Step 2: Matching descriptor vectors with a FLANN based matcher
    // Since SURF is a floating-point descriptor NORM_L2 is used
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE_SL2);
    vector<vector<DMatch>> knn_matches;
    matcher->knnMatch( descriptors1, descriptors2, knn_matches, 2 );
    //-- Filter matches using the Lowe's ratio test
    const float ratio_thresh = 0.7f;
    vector<DMatch> good_matches;
    for (size_t i = 0; i < knn_matches.size(); i++)
    {
        if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance)
        {
            good_matches.push_back(knn_matches[i][0]);
        }
    }
    //-- Draw matches
    Mat img_matches;
    drawMatches( img1, keypoints1, img2, keypoints2, good_matches, img_matches, Scalar::all(-1),
                 Scalar::all(-1), vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );
    //-- Show detected matches
    resize(img_matches, img_matches, Size(), .2, .2);
    imshow("Good Matches", img_matches);
    waitKey();
    return 0;
}
