//
// Created by lihao on 19-7-9.
//

#include "Astar.h"
#include <ros/ros.h>
namespace pathplanning{

void Astar::InitAstar(Mat& _Map, AstarConfig _config)
{
    Mat Mask;
    InitAstar(_Map, Mask, _config);
}

void Astar::InitAstar(Mat& _Map, Mat& Mask, AstarConfig _config)
{
    char neighbor8[8][2] = {
            {-1, -1}, {-1, 0}, {-1, 1},
            {0, -1},            {0, 1},
            {1, -1},   {1, 0},  {1, 1}
    };

    Map = _Map;
    config = _config;
    cout<<("IN Before!\n");
    neighbor = Mat(8, 2, CV_8S, neighbor8).clone();
ROS_INFO("IN After!\n");
    MapProcess(Mask);
}

void Astar::PathPlanning(Point _startPoint, Point _targetPoint, vector<Point>& path)
{
    // Get variables
    startPoint = _startPoint;
    targetPoint = _targetPoint;

    // Path Planning
    Node* TailNode = FindPath();
    GetPath(TailNode, path);
}

void Astar::DrawPath(Mat& _Map, vector<Point>& path, InputArray Mask, Scalar color,
        int thickness, Scalar maskcolor)
{
    if(path.empty())
    {
        cout << "Path is empty!" << endl;
        return;
    }
    _Map.setTo(maskcolor, Mask);
    for(auto it:path)
    {
        rectangle(_Map, it, it, color, thickness);
    }
}

void Astar::MapProcess(Mat& Mask)
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _Map = Map.clone();
ROS_INFO("IN Before!\n");
    // Transform RGB to gray image
    if(_Map.channels() == 3)
    {
        cvtColor(_Map.clone(), _Map, CV_BGR2GRAY);
    }

    // Binarize
    if(config.OccupyThresh < 0)
    {
        threshold(_Map.clone(), _Map, 0, 255, CV_THRESH_OTSU);
    } else
    {
        threshold(_Map.clone(), _Map, config.OccupyThresh, 255, CV_THRESH_BINARY);
    }
ROS_INFO("IN After!\n");
    // Inflate
    Mat src = _Map.clone();
    if(config.InflateRadius > 0)
    {
        Mat se = getStructuringElement(MORPH_ELLIPSE, Size(2 * config.InflateRadius, 2 * config.InflateRadius));
        erode(src, _Map, se);
    }

    // Get mask
    bitwise_xor(src, _Map, Mask);

    // Initial LabelMap
    ROS_INFO("IN Bfter!\n");
    LabelMap = Mat::zeros(height, width, CV_8UC1);
    for(int y=0;y<height;y++)
    {
        for(int x=0;x<width;x++)
        {
            if(_Map.at<uchar>(y, x) == 0)
            {
                LabelMap.at<uchar>(y, x) = obstacle;
            }
            else
            {
                LabelMap.at<uchar>(y, x) = free;
            }
        }
    }
    ROS_INFO("IN After!\n");
}

Node* Astar::FindPath()
{
    int width = Map.cols;
    int height = Map.rows;
    Mat _LabelMap = LabelMap.clone();

    // Add startPoint to OpenList
    OpenList.clear();
    OpenList.push_back(new Node(startPoint));
    _LabelMap.at<uchar>(startPoint.y, startPoint.x) = inOpenList;

    while(!OpenList.empty())
    {
        // Find the node with least F value
        Node* CurNode = OpenList[0];
        int index = 0;
        int length = OpenList.size();
        for(int i = 0;i < length;i++)
        {
            if(OpenList[i]->F < CurNode->F)
            {
                CurNode = OpenList[i];
                index = i;
            }
        }
        int curX = CurNode->point.x;
        int curY = CurNode->point.y;
        OpenList.erase(OpenList.begin() + index);       // Delete CurNode from OpenList
        _LabelMap.at<uchar>(curY, curX) = inCloseList;

        // Determine whether arrive the target point
        if(curX == targetPoint.x && curY == targetPoint.y)
        {
            return CurNode; // Find a valid path
        }

        // Traversal the neighborhood
        for(int k = 0;k < neighbor.rows;k++)
        {
            int y = curY + neighbor.at<char>(k, 0);
            int x = curX + neighbor.at<char>(k, 1);
            if(x < 0 || x >= width || y < 0 || y >= height)
            {
                continue;
            }
            if(_LabelMap.at<uchar>(y, x) == free || _LabelMap.at<uchar>(y, x) == inOpenList)
            {
                // Determine whether a diagonal line can pass
                bool walkable = true;
                if(y == curY - 1 && _LabelMap.at<uchar>(curY - 1, curX) == obstacle)
                {
                    walkable = false;
                }
                else if(y == curY + 1 && _LabelMap.at<uchar>(curY + 1, curX) == obstacle)
                {
                    walkable = false;
                }

                if(x == curX - 1 && _LabelMap.at<uchar>(curY, curX - 1) == obstacle)
                {
                    walkable = false;
                }
                else if(x == curX + 1 && _LabelMap.at<uchar>(curY, curX + 1) == obstacle)
                {
                    walkable = false;
                }
                if(!walkable)
                {
                    continue;
                }

                // Calculate G, H, F value
                int addG, G, H, F;
                if(abs(x - curX) == 1 && abs(y - curY) == 1)
                {
                    addG = 14;
                }
                else
                {
                    addG = 10;
                }
                G = CurNode->G + addG;
                if(config.Euclidean)
                {
                    int dist2 = (x - targetPoint.x) * (x - targetPoint.x) + (y - targetPoint.y) * (y - targetPoint.y);
                    H = round(10 * sqrt(dist2));
                }
                else
                {
                    H = 10 * (abs(x - targetPoint.x) + abs(y - targetPoint.y));
                }
                F = G + H;

                // Update the G, H, F value of node
                if(_LabelMap.at<uchar>(y, x) == free)
                {
                    Node* node = new Node();
                    node->point = Point(x, y);
                    node->parent = CurNode;
                    node->G = G;
                    node->H = H;
                    node->F = F;
                    OpenList.push_back(node);
                    _LabelMap.at<uchar>(y, x) = inOpenList;
                }
                else // _LabelMap.at<uchar>(y, x) == inOpenList
                {
                    // Find the node
                    Node* node = NULL;
                    int length = OpenList.size();
                    for(int i = 0;i < length;i++)
                    {
                        if(OpenList[i]->point.x ==  x && OpenList[i]->point.y ==  y)
                        {
                            node = OpenList[i];
                            break;
                        }
                    }
                    if(G < node->G)
                    {
                        node->G = G;
                        node->F = F;
                        node->parent = CurNode;
                    }
                }
            }
        }
    }

    return NULL; // Can not find a valid path
}

void Astar::GetPath(Node* TailNode, vector<Point>& path)
{
    PathList.clear();
    path.clear();

    // Save path to PathList
    Node* CurNode = TailNode;
    while(CurNode != NULL)
    {
        PathList.push_back(CurNode);
        CurNode = CurNode->parent;
    }

    // Save path to vector<Point>
    int length = PathList.size();
    for(int i = 0;i < length;i++)
    {
        path.push_back(PathList.back()->point);
        PathList.pop_back();
    }
}

}