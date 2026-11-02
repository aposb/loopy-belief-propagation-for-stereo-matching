// Loopy Belief Propagation for Stereo Matching
//

#include <iostream>
#include <cstdint>
#include <vector>
#include <chrono>
#include <opencv2/opencv.hpp>

using namespace std;
using namespace cv;

// Message update schedules
#define SYNCHRONOUS 1    // Synchronous
#define BIPARTITE 2      // Bipartite
#define BP_M 3           // BP-M (Accelerated)
#define BP_S 4           // BP-S
#define BYROW1 5         // ByRow1
#define BYROW2 6         // ByRow2

// Select message update schedule to use
#define MESSAGE_UPDATE_SCHEDULE BIPARTITE

// Belief computation methods
#define STANDARD_BELIEF_COMPUTATION 1        // Standard belief computation
#define MESSAGESONLY_BELIEF_COMPUTATION 2    // Without dataCost (larger energy but better results)

// Select belief computation method to use
#define BELIEF_COMPUTATION_METHOD MESSAGESONLY_BELIEF_COMPUTATION

typedef vector<uint8_t> CostVector;
typedef vector<vector<CostVector>> CostVolume;
typedef vector<vector<int>> CostTable;

int width, height, levels, iterations, lambda, truncation;
Mat leftImage, rightImage, disparityMap, disparityImage;
CostTable smoothnessCost;
CostVolume dataCost, msgUp, msgDown, msgRight, msgLeft;
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
CostVolume msgUp2, msgDown2, msgRight2, msgLeft2;
#endif

CostVolume initializeCostVolume();
void sendMessageUp(int x, int y);
void sendMessageDown(int x, int y);
void sendMessageRight(int x, int y);
void sendMessageLeft(int x, int y);
void createMessage(CostVector& msgData, CostVector& msgIn1, CostVector& msgIn2, CostVector& msgIn3, CostVector& msgOut);
int computeDataCost(int x, int y, int label);
int computeSmoothnessCost(int label1, int label2);
int findBestAssignment(int x, int y);
int computeBelief(int x, int y, int label);
int computeEnergy();

int main()
{
	levels = 16;
	iterations = 50;
	lambda = 5;
	truncation = 2;

	// Start timer
	auto start = chrono::steady_clock::now();

	// Read stereo image
	leftImage = imread("left.png", IMREAD_GRAYSCALE);
	rightImage = imread("right.png", IMREAD_GRAYSCALE);

	// Use gaussian filter
	GaussianBlur(leftImage, leftImage, Size(5, 5), 0.68);
	GaussianBlur(rightImage, rightImage, Size(5, 5), 0.68);

	// Get image size
	width = leftImage.cols;
	height = leftImage.rows;

	// Cache data cost
	dataCost = initializeCostVolume();
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
			for (int i = 0; i < levels; i++)
				dataCost[y][x][i] = computeDataCost(x, y, i);

	// Cache smoothness cost
	smoothnessCost = CostTable(levels, vector<int>(levels));
	for (int i = 0; i < levels; i++)
		for (int j = 0; j < levels; j++)
			smoothnessCost[i][j] = computeSmoothnessCost(i, j);

	// Initialize messages
	msgUp = initializeCostVolume();
	msgDown = initializeCostVolume();
	msgRight = initializeCostVolume();
	msgLeft = initializeCostVolume();
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
	msgUp2 = initializeCostVolume();
	msgDown2 = initializeCostVolume();
	msgRight2 = initializeCostVolume();
	msgLeft2 = initializeCostVolume();
#endif

	// Initialize disparity map
	disparityMap = Mat::zeros(height, width, CV_8U);

	// Display table
	cout << "Iter.\tEnergy" << endl;
	cout << "--------------" << endl;

	// Start iterations
	for (int iter = 1; iter <= iterations; iter++)
	{
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
		// Update messages using Synchronous message update schedule
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				if (x < width - 1)
					sendMessageRight(x, y);
				if (x > 0)
					sendMessageLeft(x, y);
				if (y < height - 1)
					sendMessageDown(x, y);
				if (y > 0)
					sendMessageUp(x, y);
			}
		msgLeft = msgLeft2;
		msgRight = msgRight2;
		msgUp = msgUp2;
		msgDown = msgDown2;
#elif MESSAGE_UPDATE_SCHEDULE == BIPARTITE
		// Update messages using Bipartite message update schedule
		for (int i = 0; i < 2; i++)
			for (int y = 0; y < height; y++)
				for (int x = (y + i) % 2; x < width; x += 2)
				{
					if (y > 0)
						sendMessageUp(x, y);
					if (y < height - 1)
						sendMessageDown(x, y);
					if (x < width - 1)
						sendMessageRight(x, y);
					if (x > 0)
						sendMessageLeft(x, y);
				}
#elif MESSAGE_UPDATE_SCHEDULE == BP_M
		// Update messages using BP-M (Accelerated) message update schedule
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width - 1; x++)
				sendMessageRight(x, y);
		for (int y = 0; y < height; y++)
			for (int x = width - 1; x >= 1; x--)
				sendMessageLeft(x, y);
		for (int x = 0; x < width; x++)
			for (int y = 0; y < height - 1; y++)
				sendMessageDown(x, y);
		for (int x = 0; x < width; x++)
			for (int y = height - 1; y >= 1; y--)
				sendMessageUp(x, y);
#elif MESSAGE_UPDATE_SCHEDULE == BP_S
		// Update messages using BP-S message update schedule
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				if (x < width - 1)
					sendMessageRight(x, y);
				if (y < height - 1)
					sendMessageDown(x, y);
			}
		for (int y = height - 1; y >= 0; y--)
			for (int x = width - 1; x >= 0; x--)
			{
				if (x > 0)
					sendMessageLeft(x, y);
				if (y > 0)
					sendMessageUp(x, y);
			}
#elif MESSAGE_UPDATE_SCHEDULE == BYROW1
		// Update messages using ByRow1 message update schedule
		for (int y = 0; y < height; y++)
		{
			for (int x = 0; x < width; x++)
			{
				if (x < width - 1)
					sendMessageRight(x, y);
				if (y < height - 1)
					sendMessageDown(x, y);
			}
			for (int x = width - 1; x >= 0; x--)
			{
				if (x > 0)
					sendMessageLeft(x, y);
				if (y > 0)
					sendMessageUp(x, y);
			}
		}
#elif MESSAGE_UPDATE_SCHEDULE == BYROW2
		// Update messages using ByRow2 message update schedule
		for (int i = 1; i >= 0; i--)
			for (int y = i; y < height; y += 2)
			{
				for (int x = 0; x < width; x++)
				{
					if (x < width - 1)
						sendMessageRight(x, y);
					if (y < height - 1)
						sendMessageDown(x, y);
				}
				for (int x = width - 1; x >= 0; x--)
				{
					if (x > 0)
						sendMessageLeft(x, y);
					if (y > 0)
						sendMessageUp(x, y);
				}
			}
#endif

		// Update disparity map
		for (int y = 0; y < height; y++)
			for (int x = 0; x < width; x++)
			{
				int label = findBestAssignment(x, y);
				disparityMap.at<uchar>(y, x) = label;
			}

		// Update disparity image
		int scaleFactor = 256 / levels;
		disparityMap.convertTo(disparityImage, CV_8U, scaleFactor);

		// Show energy
		int energy = computeEnergy();
		cout << iter << "\t" << energy << endl;

		// Show disparity image
		namedWindow("Disparity Image", WINDOW_NORMAL);
		imshow("Disparity Image", disparityImage);
		waitKey(1);
	}

	// Save disparity image
	imwrite("disparity.png", disparityImage);

	// Stop timer
	auto end = chrono::steady_clock::now();
	auto diff = end - start;
	cout << "\nRunning Time: " << chrono::duration<double, milli>(diff).count() << " ms" << endl;

	waitKey(0);

	return 0;
}

CostVolume initializeCostVolume()
{
	return CostVolume(height, vector<CostVector>(width, CostVector(levels, 0)));
}

void sendMessageUp(int x, int y)
{
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
	CostVector& msgOut = msgDown2[y - 1][x];
#else
	CostVector& msgOut = msgDown[y - 1][x];
#endif
	createMessage(dataCost[y][x], msgDown[y][x], msgRight[y][x], msgLeft[y][x], msgOut);
}

void sendMessageDown(int x, int y)
{
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
	CostVector& msgOut = msgUp2[y + 1][x];
#else
	CostVector& msgOut = msgUp[y + 1][x];
#endif
	createMessage(dataCost[y][x], msgUp[y][x], msgRight[y][x], msgLeft[y][x], msgOut);
}

void sendMessageRight(int x, int y)
{
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
	CostVector& msgOut = msgLeft2[y][x + 1];
#else
	CostVector& msgOut = msgLeft[y][x + 1];
#endif
	createMessage(dataCost[y][x], msgUp[y][x], msgDown[y][x], msgLeft[y][x], msgOut);
}

void sendMessageLeft(int x, int y)
{
#if MESSAGE_UPDATE_SCHEDULE == SYNCHRONOUS
	CostVector& msgOut = msgRight2[y][x - 1];
#else
	CostVector& msgOut = msgRight[y][x - 1];
#endif
	createMessage(dataCost[y][x], msgUp[y][x], msgDown[y][x], msgRight[y][x], msgOut);
}

void createMessage(CostVector& msgData, CostVector& msgIn1, CostVector& msgIn2, CostVector& msgIn3, CostVector& msgOut)
{
	static vector<int> inputVector = vector<int>(levels);
	static vector<int> outputVector = vector<int>(levels);

	for (int i = 0; i < levels; i++)
		inputVector[i] = msgData[i] + msgIn1[i] + msgIn2[i] + msgIn3[i];

	// Create message
	for (int i = 0; i < levels; i++)
	{
		int minCost = INT_MAX;
		for (int j = 0; j < levels; j++)
		{
			int cost = inputVector[j] + smoothnessCost[i][j];
			if (cost < minCost)
				minCost = cost;
		}
		outputVector[i] = minCost;
	}

	// Normalize message
	int minCost = INT_MAX;
	for (int i = 0; i < levels; i++)
		if (outputVector[i] < minCost)
			minCost = outputVector[i];
	for (int i = 0; i < levels; i++)
		msgOut[i] = min(outputVector[i]-minCost, 255);
}

int computeDataCost(int x, int y, int label)
{
	int leftPixel = leftImage.at<uchar>(y, x);
	int rightPixel = (x >= label) ? rightImage.at<uchar>(y, x - label) : 0;

	return abs(leftPixel - rightPixel);
}

int computeSmoothnessCost(int label1, int label2)
{
	return lambda * min(abs(label1 - label2), truncation);
}

int findBestAssignment(int x, int y)
{
	int label, minCost = INT_MAX;
	for (int i = 0; i < levels; i++)
	{
		int cost = computeBelief(x, y, i);
		if (cost < minCost)
		{
			label = i;
			minCost = cost;
		}
	}

	return label;
}

int computeBelief(int x, int y, int label)
{
#if BELIEF_COMPUTATION_METHOD == STANDARD_BELIEF_COMPUTATION
	return dataCost[y][x][label] + msgUp[y][x][label] + msgDown[y][x][label] + msgRight[y][x][label] + msgLeft[y][x][label];
#elif BELIEF_COMPUTATION_METHOD == MESSAGESONLY_BELIEF_COMPUTATION
	return msgUp[y][x][label] + msgDown[y][x][label] + msgRight[y][x][label] + msgLeft[y][x][label];
#endif
}

int computeEnergy()
{
	int energy = 0;
	for (int y = 0; y < height; y++)
		for (int x = 0; x < width; x++)
		{
			int label1 = disparityMap.at<uchar>(y, x);
			energy += dataCost[y][x][label1];
			if (x < width - 1)
			{
				int label2 = disparityMap.at<uchar>(y, x + 1);
				energy += smoothnessCost[label1][label2];
			}
			if (y < height - 1)
			{
				int label3 = disparityMap.at<uchar>(y + 1, x);
				energy += smoothnessCost[label1][label3];
			}
		}

	return energy;
}
