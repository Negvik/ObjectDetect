#include <opencv.hpp>
#include <cmath>
#include <iostream>
#include <set>
#include <windows.h>

using namespace std;
using namespace cv;

void initUart(HANDLE *hUart, LPCTSTR portName);
void binarization(Mat img, Mat &out, int hueThrMin, int hueThrMax);
void otsuThreshold(Mat img, Mat &out);
int objDetect(Mat img, vector<vector <Point>> &regList);
Point_<float> undistortPoint(Point_<float> distp);

// Parametrs
const int objectMinSize = 1500;
const int hueThrMin = 20;
const int hueThrMax = 35;
const float webcamVisionAngleX = 100.0f;
const float webcamVisionAngleY = 100.0f;
const int webcamID = 0;
const LPCTSTR port = (LPCTSTR)"COM7";

const float k1 = 0.116261;
const float k2 = -0.604692;
const float k3 = 0.772528;
const float p1 = 0.00797269;
const float p2 = 0.00184329;

const float fx = 935.243;
const float fy = 906.95;
const float cx = 655.987;
const float cy = 371.992;


int main()
{
    // Initialization
    HANDLE hUart;
    initUart(&hUart, port);

    Mat hsv, img, channel[3];
    VideoCapture cap;
    cap.open(webcamID + CAP_ANY);
    if (!cap.isOpened()) {
        cerr << "ERROR";
        return -1;
    }

    cap.read(img);
    float degreePerPixelX = webcamVisionAngleX / img.cols;
    float degreePerPixelY = webcamVisionAngleY / img.rows;
    Point_<float> imgCenter = { img.cols/2, img.rows/2 };
    Point_<float> objCenter;
    Point_<float> realObjCenter;
    DWORD dwBytesWritten = 0;


    uchar c;
    while ((c=waitKey(1)) != 'e') {
        // Get image and channels
        cap.read(img);
        cvtColor(img, hsv, CV_BGR2HSV);
        split(hsv, channel);

        // Filtrate channels
        for (int y = 1; y < channel[0].rows - 1; y++) {
            for (int x = 1; x < channel[0].cols - 1; x++) {
                channel[0].at<uchar>(y, x) = (channel[0].at<uchar>(y-1, x-1) + channel[0].at<uchar>(y-1, x) + channel[0].at<uchar>(y-1, x+1) +
                                              channel[0].at<uchar>(y, x-1) + channel[0].at<uchar>(y, x) + channel[0].at<uchar>(y, x+1) +
                                              channel[0].at<uchar>(y+1, x-1) + channel[0].at<uchar>(y+1, x) + channel[0].at<uchar>(y+1, x+1)) / 9;
                channel[1].at<uchar>(y, x) = (channel[1].at<uchar>(y-1, x-1) + channel[1].at<uchar>(y-1, x) + channel[1].at<uchar>(y-1, x+1) +
                                              channel[1].at<uchar>(y, x-1) + channel[1].at<uchar>(y, x) + channel[1].at<uchar>(y, x+1) +
                                              channel[1].at<uchar>(y+1, x-1) + channel[1].at<uchar>(y+1, x) + channel[1].at<uchar>(y+1, x+1)) / 9;
                channel[2].at<uchar>(y, x) = (channel[2].at<uchar>(y-1, x-1) + channel[2].at<uchar>(y-1, x) + channel[2].at<uchar>(y-1, x+1) +
                                              channel[2].at<uchar>(y, x-1) + channel[2].at<uchar>(y, x) + channel[2].at<uchar>(y, x+1) +
                                              channel[2].at<uchar>(y+1, x-1) + channel[2].at<uchar>(y+1, x) + channel[2].at<uchar>(y+1, x+1)) / 9;
            }
        }

        // Binarization channels
        binarization(channel[0], channel[0], hueThrMin, hueThrMax);
        otsuThreshold(channel[1], channel[1]);
        otsuThreshold(channel[2], channel[2]);
        imshow("BinHue", channel[0]);
        imshow("BinSat", channel[1]);
        imshow("BinVal", channel[2]);
        for (int y = 0; y < hsv.rows; y++)
            for (int x = 0; x < hsv.cols; x++)
                if (channel[0].at<uchar>(y, x) == 255 && channel[1].at<uchar>(y, x) == 255 && channel[2].at<uchar>(y, x) == 255)
                    channel[0].at<uchar>(y, x) = 255;
                else
                    channel[0].at<uchar>(y, x) = 0;
        imshow("BIN", channel[0]);

        // Object Detection
        vector <vector <Point>> objectList;
        objectList.resize(10000);
        int objCount = objDetect(channel[0], objectList);

        for (int i = 0; i < objCount; i++) {
            long sum_x, sum_y, count;
            sum_x = sum_y = count = 0;
            if (objectList[i].size() < objectMinSize)
                continue;
            for (uint j = 0; j < objectList[i].size(); j++) {
                sum_x += objectList[i][j].x;
                sum_y += objectList[i][j].y;
                count++;
            }

            objCenter = { sum_y/count, sum_x/count };
            realObjCenter = undistortPoint(objCenter);
            circle(img, realObjCenter, 7, Scalar{200, 10, 10});
            circle(img, objCenter, 7, Scalar{10, 10, 200});
            cout << "X angle: " << (objCenter.x - imgCenter.x) * degreePerPixelX << endl;
            cout << "Y angle: " << (img.rows - objCenter.y) * degreePerPixelY << endl;
        }
        imshow("ObjectDetect", img);

        int8_t head = 125;
        WriteFile (hUart, &head , (DWORD)1,&dwBytesWritten,NULL);
        WriteFile (hUart, &objCenter.x, (DWORD)4,&dwBytesWritten,NULL);
        WriteFile (hUart, &objCenter.y, (DWORD)4,&dwBytesWritten,NULL);
    }
    return 0;
}

Point_<float> undistortPoint(Point_<float> distp)
{
    Point_<float> undistp;
    Point_<float> start;
    float r;
    undistp.x = start.x = (distp.x - cx) / fx;
    undistp.y = start.y = (distp.y - cy) / fy;
    for (int i = 0; i < 5; i++) {
        r = std::sqrt((undistp.x*undistp.x + undistp.y*undistp.y));
        undistp.x = (start.x - (p2*(r*r + 2*undistp.x*undistp.x) + 2*p1*undistp.x*undistp.y))/
                    (1 + k1*r*r + k2*r*r*r*r + k3*r*r*r*r*r*r);
        undistp.y = (start.y - (p2*(r*r + 2*undistp.y*undistp.y) + 2*p1*undistp.x*undistp.y))/
                    (1 + k1*r*r + k2*r*r*r*r + k3*r*r*r*r*r*r);

    }
    undistp.x = undistp.x*fx + cx;
    undistp.y = undistp.y*fy + cy;
    return undistp;
}

uchar getSingleBits(uchar byte)
{
    uchar count = 0;
    for (size_t i = 0; i < 8; i++)
        if ((byte >> i) & 0x1)
            count++;
    return count;
}

DWORD uartSendAngles(HANDLE uartHandler, Point obj, LPCTSTR portName)
{
    uchar head = 242;
    uchar dataFirstByte = uchar(obj.x);
    uchar dataSecondByte = uchar(obj.y);
    uchar cSum = getSingleBits(dataFirstByte) + getSingleBits(dataSecondByte);
    uchar packet[4] = { head, dataFirstByte, dataSecondByte, cSum };
    DWORD bytesWritten = 0;

    WriteFile (uartHandler, packet, (DWORD)4, &bytesWritten, NULL);
    return bytesWritten;
}

int objDetect(Mat img, vector<vector <Point>> &regList)
{
    uint16_t regionNumber = 1;
    vector <set<uint16_t>> connectionTab(10000);

    vector <vector<uint16_t>> label(img.rows);
    for (int i = 0; i < img.rows; i++) {
        label[i].resize(img.cols);
        for (int j = 0; j < img.cols; j++) {
            label[i][j] = 9999;
        }
    }

    // First pass
    for (int y = 1; y < img.rows; y++) {
        for (int x = 1; x < img.cols-1; x++) {
            if (img.at<uchar>(y, x) == 0)
                continue;
            else if (img.at<uchar>(y, x-1) == 0 && img.at<uchar>(y-1, x) == 0 &&
                     img.at<uchar>(y-1, x-1) == 0 && img.at<uchar>(y-1, x+1) == 0) {
                label[y][x] = regionNumber;
                connectionTab[regionNumber].insert(regionNumber);
                regionNumber++;
            } else {
                label[y][x] = min(min(label[y][x-1], label[y-1][x]), min(label[y-1][x-1], label[y-1][x+1]));
                connectionTab[label[y][x-1]].insert(label[y][x]);
                connectionTab[label[y-1][x]].insert(label[y][x]);
                connectionTab[label[y-1][x-1]].insert(label[y][x]);
                connectionTab[label[y-1][x+1]].insert(label[y][x]);
            }
        }
    }

    // Second pass
    regionNumber = 0;
    vector <uint16_t> newLabel(0);
    for (int y = 0; y < img.rows; y++) {
        for (int x = 0; x < img.cols; x++) {
            if (img.at<uchar>(y, x) != 0) {
                label[y][x] = *connectionTab[label[y][x]].begin();
                int lab = -1;
                for (uint i = 0; i < newLabel.size(); i++)
                    if (newLabel[i] == label[y][x]) {
                        lab = i;
                        break;
                    }
                if (lab == -1) {
                    newLabel.push_back(label[y][x]);
                    regList[newLabel.size()-1].push_back(Point{y, x});
                } else {
                    regList[lab].push_back(Point{y, x});
                }
            }
        }
    }
    return newLabel.size();
}

void imgHist(Mat img, int hist[256])
{
    for (int i = 0; i < 256; i++)
        hist[i] = 0;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            hist[img.at<uchar>(y, x)]++;
}

long allImgIntensity(Mat img)
{
    long sum = 0;
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            sum += img.at<uchar>(y, x);
    return sum;
}

void otsuThreshold(Mat img, Mat &out)
{
    long all_pixels_intensity = allImgIntensity(img);
    long all_pixels_count = img.rows*img.cols;
    long first_class_pixel_count = 0;
    double first_class_intensity = 0;

    int hist[256];
    imgHist(img, hist);

    int best_th = 0;
    double best_sigma = 0.0;
    for (int i = 0; i < 255; i++) {
        first_class_pixel_count += hist[i];
        first_class_intensity += i*hist[i];

        double first_class_prob = first_class_pixel_count / (double)all_pixels_count;
        double second_class_prob = 1.0 - first_class_prob;
        double first_class_mean = first_class_intensity / (double)first_class_pixel_count;
        double second_class_mean = (all_pixels_intensity - first_class_intensity) / (double) (all_pixels_count - first_class_pixel_count);
        double delta_mean = first_class_mean - second_class_mean;

        double sigma = first_class_prob*second_class_prob*delta_mean*delta_mean;
        if (sigma > best_sigma) {
            best_sigma = sigma;
            best_th = i;
        }
    }
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++)
            out.at<uchar>(y, x) = (img.at<uchar>(y, x) >= best_th) ? 255 : 0;
}

void binarization(Mat img, Mat &out, int thrMin, int thrMax)
{
    for (int y = 0; y < img.rows; y++)
        for (int x = 0; x < img.cols; x++) {
            out.at<uchar>(y, x) = (img.at<uchar>(y, x) >= thrMin && img.at<uchar>(y, x) <= thrMax) ? 255 : 0;
        }
}

void initUart(HANDLE *hUart, LPCTSTR portName)
{
    *hUart = CreateFile(portName, GENERIC_WRITE | GENERIC_READ, 0, 0, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, 0);
    if (*hUart == INVALID_HANDLE_VALUE)
        cerr << "UART error\n";
    DCB dcbUartParams = { 0 };
    if (!GetCommState(*hUart, &dcbUartParams))
        cerr << "getting state error\n";
    dcbUartParams.BaudRate=CBR_9600;
    dcbUartParams.ByteSize=8;
    dcbUartParams.StopBits=ONESTOPBIT;
    dcbUartParams.Parity=NOPARITY;
    if(!SetCommState(*hUart, &dcbUartParams)) {
        cerr << "error setting serial port state\n";
    }
}


