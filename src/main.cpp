#include <ros/ros.h>

#include <iostream>
#include <sys/time.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry> // for Lie algeb

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// threading
#include <thread>
#include <cmath>

#include <vector>
#include <algorithm>

#include <ctime>

#define PI 3.141592

void c__logKernel(const int &size_, const double &sigma_, cv::Mat &kernel_);
void c__imgMean(const cv::Mat &imgInput_, double &mean_);
void c__indLog(const cv::Mat &imgInput_, cv::Mat &imgIndex_, const double &respThreshold);
void c__conv2(const cv::Mat &imgInput_, const cv::Mat &kernel_, cv::Mat &imgRes_);
void c__imgStd(const cv::Mat &imgInput_, const double &mean_, double &std_);
void c__indBright(const cv::Mat &imgInput_, cv::Mat &imgIndex_, const double &respThreshold);
void c__combineInd(const cv::Mat &indLog_, const cv::Mat &indBright_, cv::Mat &ind_);
void c__imgDilate(const cv::Mat &inputInd_, cv::Mat &dilatedInd_);
void c__findVerge(const cv::Mat &inputInd_, cv::Mat &vergeInd_);


int main(int argc, char **argv)
{
	ros::init(argc, argv, "hanwha_node");
	ros::NodeHandle nh("~");

	cv::Mat imgRaw8, imgGray8, imgGray, logKernel, imgLog;
	cv::Mat indLog, indBright, ind, indDilated;

	imgRaw8 = cv::imread("/home/icslkchlap/catkin_ws/src/hanwha_algorithm/images/sunglint2_p4.png", -1);
	cv::cvtColor(imgRaw8, imgGray8, CV_BGR2GRAY); // convert to grayscale image
	imgGray8.convertTo(imgGray, CV_64FC1);		  // make double array

	clock_t start, finish;
	start = clock();
	// Do my algorithm
	c__logKernel(13, 1.4, logKernel); // LoG kernel
	cv::filter2D(imgGray, imgLog, -1, logKernel, cv::Point(-1, -1), 0);
	c__indLog(imgLog, indLog, -3.0);
	finish = clock();
	double duration = (double)(finish - start) / 1000000.0;
	printf("1. LoG : %f 초\n", duration);

	start = clock();
	double brightMean, brightStd, brightThreshold;
	c__imgMean(imgGray, brightMean);
	c__imgStd(imgGray, brightMean, brightStd);
	c__indBright(imgGray, indBright, brightMean + 1.2 * brightStd);
	finish = clock();
	duration = (double)(finish - start) / 1000000.0;
	printf("2. Brightness : %f 초\n", duration);
	start = clock();
	c__combineInd(indLog, indBright, ind);
	finish = clock();
	duration = (double)(finish - start) / 1000000.0;
	printf("3. Combine : %f 초\n", duration);
	start = clock();
	c__imgDilate(ind, indDilated);
	finish = clock();
	duration = (double)(finish - start) / 1000000.0;
	printf("4.dilate : %f 초\n", duration);

	// Sun glint rejection algorithm
	int winSize = 7;
	int halfWinSize = (winSize - 1) / 2;
	cv::Mat imgRestored, resInd, vergeInd;
	imgGray.copyTo(imgRestored); // double Mat
	indDilated.copyTo(resInd);

	start = clock();
	double windowTemp[winSize * winSize / 2] = {0};
	for (int iter = 0; iter < 10; iter++)
	{
		c__findVerge(resInd, vergeInd);
		for (int v = halfWinSize; v < imgGray.rows - halfWinSize - 1; v++)
		{
			uchar *vergeIndPtr = vergeInd.ptr<uchar>(v);
			uchar *resIndPtr = resInd.ptr<uchar>(v);
			double *imgRestoredPtr = imgRestored.ptr<double>(v);
			for (int u = halfWinSize; u < imgGray.cols - halfWinSize - 1; u++)
			{
				if (*(vergeIndPtr + u) == 255)
				{
					double medianTemp;
					std::vector<double> vectorTemp;
					for (int w = u - halfWinSize*(1 + winSize); w < u + halfWinSize * (1 + winSize) + 1; w++)
					{
						if (*(resIndPtr + w) == 255)
						{
							double temp = *(imgRestoredPtr + w);
							vectorTemp.push_back(temp);
						}
					}

					std::nth_element(vectorTemp.begin(), vectorTemp.begin() + vectorTemp.size() / 2, vectorTemp.end());
					medianTemp = vectorTemp[vectorTemp.size() / 2];
					*(imgRestoredPtr + u) = medianTemp;
					*(resIndPtr + u) = 0;
				}
			}
		}
	}
	finish = clock();
	duration = (double)(finish - start) / 1000000.0;
	printf("5.verge : %f 초\n", duration);

	cv::namedWindow("img_raw");
	cv::imshow("img_raw", imgRestored/255.0);
	cv::waitKey(0);

	return 0;
}

//////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////
void c__conv2(const cv::Mat &imgInput_, const cv::Mat &kernel_, cv::Mat &imgRes_) // TODO
{

	for (int v = 0; v < imgInput_.rows; v++)
	{
		const double *imgInputPtr = imgInput_.ptr<double>(v);
	}
}

void c__logKernel(const int &size_, const double &sigma_, cv::Mat &kernel_) // Done
{
	kernel_.create(cv::Size(size_, size_), 6);
	if ((size_ % 2) == 0)
	{
		std::cout << "Please odd number" << std::endl;
	}
	else
	{
		int halfSize = (size_ - 1) / 2;
		double invSigma4Pi = 1.0 / (sigma_ * sigma_ * sigma_ * sigma_ * PI);
		for (int i = -halfSize; i < halfSize + 1; i++)
		{
			double *kernelPtr = kernel_.ptr<double>(i + halfSize);
			for (int j = -halfSize; j < halfSize + 1; j++)
			{
				*(kernelPtr + j + halfSize) = -invSigma4Pi * (1.0 - (i * i + j * j) * 0.5 / (sigma_ * sigma_)) * exp(-(i * i + j * j) * 0.5 / (sigma_ * sigma_));
			}
		}
	}
}

void c__indLog(const cv::Mat &imgInput_, cv::Mat &imgIndex_, const double &respThreshold) // Done
{
	imgIndex_.create(cv::Size(imgInput_.cols, imgInput_.rows), CV_8UC1);
	for (int v = 0; v < imgInput_.rows; v++)
	{
		const double *imgInputPtr = imgInput_.ptr<double>(v);
		uchar *imgIndexPtr = imgIndex_.ptr<uchar>(v);
		for (int u = 0; u < imgInput_.cols; u++)
		{
			if (*(imgInputPtr + u) < respThreshold)
			{
				*(imgIndexPtr + u) = 255;
			}
			else
			{
				*(imgIndexPtr + u) = 0;
			}
		}
	}
}

void c__imgMean(const cv::Mat &imgInput_, double &mean_) // Done
{
	double sum = 0.0;
	for (int v = 0; v < imgInput_.rows; v++)
	{
		const double *imgInputPtr = imgInput_.ptr<double>(v);
		for (int u = 0; u < imgInput_.cols; u++)
			sum += *(imgInputPtr + u);
	}

	mean_ = sum / (imgInput_.rows * imgInput_.cols);
}

void c__imgStd(const cv::Mat &imgInput_, const double &mean_, double &std_) // Done
{
	double sum = 0.0;
	for (int v = 0; v < imgInput_.rows; v++)
	{
		const double *imgInputPtr = imgInput_.ptr<double>(v);
		for (int u = 0; u < imgInput_.cols; u++)
			sum += (*(imgInputPtr + u) - mean_) * (*(imgInputPtr + u) - mean_);
	}

	std_ = sqrt(sum / (imgInput_.rows * imgInput_.cols));
}

void c__indBright(const cv::Mat &imgInput_, cv::Mat &imgIndex_, const double &respThreshold) // Done
{
	imgIndex_.create(cv::Size(imgInput_.cols, imgInput_.rows), CV_8UC1);
	for (int v = 0; v < imgInput_.rows; v++)
	{
		const double *imgInputPtr = imgInput_.ptr<double>(v);
		uchar *imgIndexPtr = imgIndex_.ptr<uchar>(v);
		for (int u = 0; u < imgInput_.cols; u++)
		{
			if (*(imgInputPtr + u) > respThreshold)
			{
				*(imgIndexPtr + u) = 255;
			}
			else
			{
				*(imgIndexPtr + u) = 0;
			}
		}
	}
}

void c__combineInd(const cv::Mat &indLog_, const cv::Mat &indBright_, cv::Mat &ind_)
{
	ind_.create(cv::Size(indLog_.cols, indLog_.rows), CV_8UC1);
	for (int v = 0; v < indLog_.rows; v++)
	{
		const uchar *indLogPtr = indLog_.ptr<uchar>(v);
		const uchar *indBrightPtr = indBright_.ptr<uchar>(v);
		uchar *indPtr = ind_.ptr<uchar>(v);

		for (int u = 0; u < indLog_.cols; u++)
		{
			if (*(indLogPtr + u) == 255 && *(indBrightPtr + u) == 255)
			{
				*(indPtr + u) = 255;
			}
			else
			{
				*(indPtr + u) = 0;
			}
		}
	}
}

void c__imgDilate(const cv::Mat &inputInd_, cv::Mat &dilatedInd_) // DONE
{
	dilatedInd_.create(cv::Size(inputInd_.cols, inputInd_.rows), CV_8UC1);
	dilatedInd_ = cv::Mat::zeros(inputInd_.rows, inputInd_.cols, CV_8UC1);

	for (int v = 1; v < inputInd_.rows - 1; v++)
	{
		const uchar *inputIndPtr = inputInd_.ptr<uchar>(v);
		uchar *dilatedIndPtrTop = dilatedInd_.ptr<uchar>(v + 1);
		uchar *dilatedIndPtrMid = dilatedInd_.ptr<uchar>(v);
		uchar *dilatedIndPtrBot = dilatedInd_.ptr<uchar>(v - 1);

		for (int u = 1; u < inputInd_.cols - 1; u++)
		{
			if (*(inputIndPtr + u) == 255)
			{
				*(dilatedIndPtrTop + u) = 255;
				*(dilatedIndPtrMid + u) = 255;
				*(dilatedIndPtrMid + u + 1) = 255;
				*(dilatedIndPtrMid + u - 1) = 255;
				*(dilatedIndPtrBot + u) = 255;
			}
		}
	}
}

void c__findVerge(const cv::Mat &inputInd_, cv::Mat &vergeInd_) // DONE
{
	inputInd_.copyTo(vergeInd_);
	for (int v = 1; v < inputInd_.rows - 1; v++)
	{
		const uchar *inputIndPtrTop = inputInd_.ptr<uchar>(v - 1);
		const uchar *inputIndPtrMid = inputInd_.ptr<uchar>(v);
		const uchar *inputIndPtrBot = inputInd_.ptr<uchar>(v + 1);
		uchar *vergeIndPtr = vergeInd_.ptr<uchar>(v);

		for (int u = 1; u < inputInd_.cols - 1; u++)
		{
			if (*(inputIndPtrMid + u) == 255 && (*(inputIndPtrMid + u + 1) == 0 || *(inputIndPtrMid + u - 1) == 0 || *(inputIndPtrTop + u) == 0 || *(inputIndPtrBot + u) == 0))
			{
				*(vergeIndPtr + u) = 255;
			}
			else
			{
				*(vergeIndPtr + u) = 0;
			}
		}
	}
}