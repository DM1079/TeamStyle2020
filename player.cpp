#include "API.h"
#include "Constant.h"
#include "player.h"
#include <iostream>
#include "OS_related.h"
#include <vector>
#include <list>
#include <math.h>
#define PI 3.1415926535
using namespace THUAI3;
Protobuf::Talent initTalent = Protobuf::Talent::None;//指定人物天赋。选手代码必须定义此变量，否则报错

const int kCost1 = 10; //直移一格消耗
const int kCost2 = 9999; //斜移一格消耗
const char dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };
chrono::milliseconds time_200ms(200);
chrono::milliseconds time_50ms(200);
struct Point {
    int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H; //F=G+H	
    Point* parent; //parent的坐标，这里没有用指针，从而简化代码
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化	
    {
    }
};

class Astar
{
public:
    Astar(const std::vector<std::vector<short>>& _maze)
    {
        maze = _maze;
        initmaze = _maze;
    }
    void setmap(int x, int y, int label)
    {
        maze[x][y] = label;//临时改变map
    }
    void resetmap()
    {
        maze = initmaze;//恢复初始map
    }
    std::list<char> GetPath(double x, double y, bool isIgnoreCorner);
    std::list<char> gotolist; //qweadzxc
    std::vector<std::vector<short>> maze;
private:
    Point* findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner);
    std::vector<Point*> getSurroundPoints(const Point* point, bool isIgnoreCorner) const;
    bool isCanreach(const Point* point, const Point* target, bool isIgnoreCorner) const; //判断某点是否可以用于下一步判断	
    Point* isInList(const std::list<Point*>& list, const Point* point) const; //判断开启/关闭列表中是否包含某点	
    Point* getLeastFpoint(); //从开启列表中返回F值最小的节点	
                             //计算FGH值	
    int calcG(Point* temp_start, Point* point);
    int calcH(Point* point, Point* end);
    int calcF(Point* point);
private:
    std::list<Point*> openList;  //开启列表	
    std::list<Point*> closeList; //关闭列表
    std::vector<std::vector<short>> initmaze;
};

int Astar::calcG(Point* temp_start, Point* point) {
    int extraG = (abs(point->x - temp_start->x) + abs(point->y - temp_start->y)) == 1 ? kCost1 : kCost2;
    int parentG = point->parent == NULL ? 0 : point->parent->G; //如果是初始节点，则其父节点是空	
    return parentG + extraG;
}

int Astar::calcH(Point* point, Point* end) {
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^	
    return sqrt((double)(end->x - point->x) * (double)(end->x - point->x) + (double)(end->y - point->y) * (double)(end->y - point->y)) * kCost1;
}

int Astar::calcF(Point* point) {
    return point->G + point->H;
}


Point* Astar::getLeastFpoint() {
    if (!openList.empty()) {
        auto resPoint = openList.front();
        for (auto& point : openList)
            if (point->F < resPoint->F)
                resPoint = point;
        return resPoint;
    }
    return NULL;
}


Point* Astar::findPath(Point& startPoint, Point& endPoint, bool isIgnoreCorner)
{
    openList.push_back(new Point(startPoint.x, startPoint.y)); //置入起点,拷贝开辟一个节点，内外隔离	
    while (!openList.empty()) {
        auto curPoint = getLeastFpoint(); //找到F值最小的点		
        openList.remove(curPoint); //从开启列表中删除		
        closeList.push_back(curPoint); //放到关闭列表		//1,找到当前周围八个格中可以通过的格子		
        auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);
        for (auto& target : surroundPoints)
        {
            //2,对某一个格子，如果它不在开启列表中，加入到开启列表，设置当前格为其父节点，计算F G H			
            if (!isInList(openList, target))
            {
                target->parent = curPoint;
                target->G = calcG(curPoint, target);
                target->H = calcH(target, &endPoint);
                target->F = calcF(target);
                openList.push_back(target);
            }			//3，对某一个格子，它在开启列表中，计算G值, 如果比原来的大, 就什么都不做, 否则设置它的父节点为当前点,并更新G和F			
            else
            {
                int tempG = calcG(curPoint, target);
                if (tempG < target->G)
                {
                    target->parent = curPoint;
                    target->G = tempG;
                    target->F = calcF(target);
                }
            }
            Point* resPoint = isInList(openList, &endPoint);
            if (resPoint)
                return resPoint; //返回列表里的节点指针，不要用原来传入的endpoint指针，因为发生了深拷贝		
        }
    }
    return NULL;
}
int sgn(double x)
{
    if (x > 0) return 1;
    if (x == 0) return 0;
    if (x < 0) return -1;

}
std::list<char> Astar::GetPath(double x, double y, bool isIgnoreCorner)
{
    Point startPoint((int)(PlayerInfo.position.x), (int)(PlayerInfo.position.y));	//网上往右是+
    cout << "start:"<< PlayerInfo.position.x <<","<< PlayerInfo.position.y <<endl;
    Point endPoint(x, y);
	//A*算法找寻路径	
    Point* result = findPath(startPoint, endPoint, isIgnoreCorner);	
    std::list<Point*> path;	//返回路径，如果没找到路径，返回空链表	
    while (result) {
        path.push_front(result);
        result = result->parent;
    }     // 清空临时开闭列表，防止重复执行GetPath导致结果异常   
    openList.clear();
    closeList.clear();
    //return path
    list<Point*>::iterator p;
    list<char>::iterator lp;
    gotolist.clear();
    int i = 1;
    for (p = path.begin();;)
    {
        Point* pre = *p;
        p++;
        int movex = (*p)->x - pre->x;//dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };
        int movey = (*p)->y - pre->y;
        char c = dir[movey + 1][movex + 1];
        gotolist.push_back(c);//例如dir[2][1]=w,dir[0][2]=c
        if (i == path.size() - 1)break;
        i++;
    }
    return gotolist;
}

Point* Astar::isInList(const std::list<Point*>& list, const Point* point) const
{	//判断某个节点是否在列表中，这里不能比较指针，因为每次加入列表是新开辟的节点，只能比较坐标	
    for (auto p : list)
        if (p->x == point->x && p->y == point->y)
            return p;
    return NULL;
}

bool Astar::isCanreach(const Point* point, const Point* target, bool isIgnoreCorner) const {
    if (target->x<0 || target->x>maze.size() - 1 || target->y<0 || target->y>maze[0].size() - 1 || maze[target->x][target->y] == 1 || target->x == point->x && target->y == point->y || isInList(closeList, target)) //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false		
        return false;
    else
    {
        if (abs(point->x - target->x) + abs(point->y - target->y) == 1) //非斜角可以			
            return true;
        else
        {			//斜对角要判断是否绊住			
            if (maze[point->x][target->y] == 0 && maze[target->x][point->y] == 0)
                return true;
            else
                return isIgnoreCorner;
        }
    }
}
//只保留上下左右四个方向，不然太容易卡住了呜呜呜
std::vector<Point*> Astar::getSurroundPoints(const Point* point, bool isIgnoreCorner) const {
    std::vector<Point*> surroundPoints;
    int x = point->x - 1;
    int y = point->y;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    x = point->x + 1;
    y = point->y;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    x = point->x;
    y = point->y + 1;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    x = point->x;
    y = point->y - 1;
    if (isCanreach(point, new Point(x, y), isIgnoreCorner))
        surroundPoints.push_back(new Point(x, y));
    return surroundPoints;
}
const std::vector<std::vector<short>> init_mapinfo_block =//0表示可以通过，1表示障碍物，这个和原版的地图是一样的，就是把0以外的数字都改成了1.
{
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 1, 1, 1, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} };

Astar astar(init_mapinfo_block);

void move_dir(char c,int i) //重载一下，i=1时只移动50ms表示时间，主要用于修改朝向。
{
    if (i == 1) {
        switch (c)
        {
        case 'd':move(Protobuf::Direction::Right, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'e':move(Protobuf::Direction::RightUp, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'w':move(Protobuf::Direction::Up, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'q':move(Protobuf::Direction::LeftUp, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'a':move(Protobuf::Direction::Left, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'z':move(Protobuf::Direction::LeftDown, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'x':move(Protobuf::Direction::Down, 50);
            this_thread::sleep_for(time_50ms);
            break;
        case 'c':move(Protobuf::Direction::RightDown, 50);
            this_thread::sleep_for(time_50ms);
            break;

        default:
            break;
        }
    }

}

void move_dir(char c) //按照c中表示的方向移动
{
    switch (c)
    {
    case 'd':move(Protobuf::Direction::Right, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'e':move(Protobuf::Direction::RightUp, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;
    case 'w':move(Protobuf::Direction::Up, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'q':move(Protobuf::Direction::LeftUp, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;
    case 'a':move(Protobuf::Direction::Left, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'z':move(Protobuf::Direction::LeftDown, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;
    case 'x':move(Protobuf::Direction::Down, 200);
        this_thread::sleep_for(time_200ms);
        break;
    case 'c':move(Protobuf::Direction::RightDown, 265);
        this_thread::sleep_for(time_200ms);
        this_thread::sleep_for(time_50ms);
        break;

    default:
        break;
    }

}

double calcdis(Point &point, Point &end) {
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^	
    return sqrt((double)(end.x - point.x) * (double)(end.x - point.x) + (double)(end.y - point.y) * (double)(end.y - point.y));
}

////////////////////////////////////////////////////////////////////////////////

Point findnearfood()//找最近的食物生成点
{
    Point Point1(25, 4), Point2(42, 39), Point3(7, 40);//三个食物生成点下方一格，这样统一再往上走一步就好
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    if (dis1 <= dis2 && dis1 <= dis3) return Point1;
    if (dis2 <= dis1 && dis2 <= dis3) return Point2;
    if (dis3 <= dis1 && dis3 <= dis2) return Point3;
}

Point findsecondfood()//找第二近的食物生成点
{
    Point Point1(25, 4), Point2(42, 39), Point3(7, 40);//三个食物生成点下方一格，这样统一再往上走一步就好
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    if ((dis1 <= dis2) != (dis1 < dis3)) return Point1;
    if ((dis2 <= dis1) != (dis2 < dis3)) return Point2;
    if ((dis3 <= dis1) != (dis3 < dis2)) return Point3;
}

char dir_4[5] = { 0,'a','w','d','x' };//用这个数组存一下最后一步的方向
double angle_4[5] = {0, PI,PI/2,0,-PI/2 };//用这个数组存一下最后扔的方向
int cooklabel[5][5] = { {}, {8,25,1,4,0}, {25,37,2,2,0}, {40,28,3,3,0}, {33,17,4,2,0} };
//第一行都是空着备用。
//【灶台边空地坐标xy,灶台编号，最后一步的方向,label(检查的时候方便吧大概)】
int label;//记录“据点”是第几个灶台


int findnearcook()//找最近的灶台
{
    Point Point1(8,25), Point2(25,37), Point3(40,28), Point4(33,17);//四个灶台
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    double dis4 = calcdis(Point4, Pos);
    if (dis1 <= dis2 && dis1 <= dis3 && dis1 <= dis4) return 1;
    if (dis2 <= dis1 && dis2 <= dis3 && dis2 <= dis4) return 2;
    if (dis3 <= dis1 && dis3 <= dis2 && dis3 <= dis4) return 3;
    if (dis4 <= dis1 && dis4 <= dis2 && dis4 <= dis3) return 4;
}

//从字符获得下一步的x和y怎么变化
int nextx(char c)
{
    if (c == 'd')return 1;
    if (c == 'a')return -1;
    else return 0;
}

int nexty(char c)
{
    if (c == 'w')return 1;
    if (c == 'x')return -1;
    else return 0;
}

void gotodest(Point &dest)
{
    list<char>::iterator lp;
    int x = dest.x;
    int y = dest.y;
    //用Getpath获得一个由字符组成的链表，字符代表移动方向，每一个是用200ms的时间走一格。
    cout << "speed" << PlayerInfo.moveSpeed << endl;
    if (astar.maze[dest.x][dest.y] == 1)//首先检测目标是否是障碍物，如果是，搜索周围四格中不是障碍物的点作为目标。
    {
        cout << "change dest" << endl;
        if (astar.maze[dest.x + 1][dest.y] == 0) {
            x = dest.x + 1;
            y = dest.y;
        }
        if (astar.maze[dest.x - 1][dest.y] == 0) {
            x = dest.x - 1;
            y = dest.y;
        }
        if (astar.maze[dest.x][dest.y+1] == 0) {
            x = dest.x;
            y = dest.y + 1;
        }
        if (astar.maze[dest.x + 1][dest.y-1] == 0) {
            x = dest.x;
            y = dest.y-1;
        }
    }
    if ((int)PlayerInfo.position.x == x && (int)PlayerInfo.position.y == y)//如果已经在目的地，没必要移动。
    {
        cout << "no need move" << endl;
        return;
    }
    while (1) {
        for (auto& lp : astar.GetPath(x, y, false))
        {
            double pos_prex = PlayerInfo.position.x;
            double pos_prey = PlayerInfo.position.y;
            double pos_nextx = pos_prex + nextx(lp);
            double pos_nexty = pos_prey + nexty(lp);
            list<Obj> l = MapInfo::get_mapcell((int)pos_nextx, (int)pos_nexty);//先检查下一个格子有没有能捡起来的或者trigger
            for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
            {
                if (i->objType == 2) cout << "dish" << i->dish << endl;
                if (i->objType == 3) cout << "tool" << i->tool << endl;
                if (i->objType == 4) cout << "trigger" << i->trigger << endl;
            }//如果有问题需要处理，还没写这里
            move_dir(lp);//处理完刚刚的问题再移动
            if (PlayerInfo.position.x == pos_prex && PlayerInfo.position.y == pos_prey) {
                cout << "error!" << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
                //char dir_4[5] = { 0,'a','w','d','x' };//用这个数组存一下最后一步的方向
                int rand1 = rand() % 4+1;//1,2,3,4
                int rand2 = rand() % 2-1;//-1,0,1  0~5
                int add12 = rand1 + rand2;
                if (add12 == 0)add12 = 4;
                if (add12 == 5) add12 = 1;
                move_dir(dir_4[rand1]);
                move_dir(dir_4[add12]);
                break;
            }//如果移动失败了，随机向某个方向移动一格，无论是否成功，都break循环，重新生成Getpath
            //cout << THUAI3::getGameTime() << endl;
        }
        if ((int)PlayerInfo.position.x == (int)x && (int)PlayerInfo.position.y == (int)y)
        {
            cout << "finish!" << endl;
            break;
        }
        //直到到达目的地再break循环。
    }
}
//走到灶台后第一件事检查黑暗料理
int throw_darkdish(int _label)//检查灶台上有无黑暗料理，如果有，尝试拾取，拾取成功就扔掉，拾取失败就标记这个灶台不行，找下一个灶台
{
    if (cooklabel[_label][4] == 1)return 0;//这个灶台是自己正在做的，不要扔黑暗料理

    int x = cooklabel[_label][0]+ nextx(dir_4[cooklabel[label][3]]);
    int y = cooklabel[_label][1] + nexty(dir_4[cooklabel[label][3]]);
    char c = dir_4[cooklabel[_label][3]];
    list<Obj> l = MapInfo::get_mapcell(x, y);
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType==3 && i->dish!=0)//如果发现灶台里面有菜
        {
            cout << "dish in cook!" << endl;
            put(0, 0, TRUE);//先把手里的东西放脚下
            move_dir(c,1);//确保朝向
            pick(FALSE, Block, 0);//尝试拾取
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish == 0)//说明没有拿到dish,也就是这个灶台正在烹饪
            {
                if (label == _label)label = 0;//如果这就是我标记的据点，那么取消标记。
                return 1;//继续跑第二近的灶台。
            }
            else 
            {
                if (PlayerInfo.dish == DarkDish)//如果拿到了黑暗料理，扔出去
                {
                    cout << "throw dark dish  "<<endl;
                    put(2, 0, TRUE);//先往右扔两格,我下次看看最多能扔多远……
                    return 2;//用这个灶台做菜，不要慌
                }
                else //不是黑暗料理！捡到宝了！
                {
                    return 3;//准备提交食物，耶
                }
            }
        }
    }
    return 2;//灶台里啥都没有，直接做菜。
}

//先写一个只看灶台一格的试试，不然太难了

int rawfood[51][3];//全局变量保平安，谁乐意传参啊hh
//每一行是 {有无食材 有1 无0，食材坐标x,食材坐标y}，如果有不止一个食材，直接覆盖，无所谓。
//第一行raw[0][0]=1,方便后面运算
//如果食材在队友手上，坐标记0，0，如果在自己手里，记50，50
//不如都先把食材扔下去吧？这样省事。

void get_all_dish(int label)//检查包括指定点（灶台）在内周围八格有无
//返回一个0-51的数组，每个数字指示mapcell中是否有该食材，第0位为1时,说明什么都没有。
{
    for (int i = 0; i <= 50; i++)
    {
        for (int j = 0; j <= 2; j++)
        {
            rawfood[i][j] = 0;//初始化
        }
    }
    rawfood[0][0] = 1;//第一行raw[0][0]=1,方便后面运算
    int x = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
    int y = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    list<Obj> l = MapInfo::get_mapcell(x, y);
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if(i->dish!=0)//
        {
            rawfood[i->dish][0] = 1;
            rawfood[i->dish][1] = (int)i->position.x;
            rawfood[i->dish][2] = (int)i->position.y;
        }
        //////////////////////////////////////////////////////////
    }

}
int get_one_dish(int x, int y)//看看食物生成点有没有食物
{
    list<Obj> l = MapInfo::get_mapcell(x, y);
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    { 
        if (i->objType == Block)
        {
            cout << "dish:" << i->dish << endl;
            return i->dish;
        }
    }
}

int pick_dish_in_block(Point &food) {//Point 是食物生成点下方一格
    if ( ((int)PlayerInfo.position.x) != food.x || ((int)PlayerInfo.position.y) != food.y) {
        gotodest(Point(food.x, food.y));
        move_dir('w');
    }
    int dish = get_one_dish((int)PlayerInfo.position.x, (int)PlayerInfo.position.y + 1);
    while (dish == 0)//如果dish=0说明没有食物,就一直等着
    {
        this_thread::sleep_for(time_50ms);
        dish = get_one_dish((int)PlayerInfo.position.x, (int)PlayerInfo.position.y + 1);
        cout << "wait for dish" << endl;
    }

    pick(FALSE, Block, 0);//是block的时候第三个随便输入,表示捡起block里的食材。
    cout << "pick finish" << endl;
    move_dir(dir_4[rand() % 4 +1]);//随机走一下，防止卡住
    return PlayerInfo.dish;
}

int  howtodolist[21][4] =//菜肴编号-20=行号
{
    {1,0,0,0},//0-20:面粉
    {20,0,0,0},//1-21：面条
    {4,20,0,0},
    {2,0,0,0},
    {3,0,0,0},
    {5,0,0,0},
    {3,4,0,0},
    {20,26,0,0},
    {11,20,0,0},
    {12,18,23,0},
    {13,14,23,0},
    {12,15,0,0},
    {14,24,0,0},
    {8,9,0,0},
    {22,11,15,0},
    {10,25,0,0},
    {13,24,0,0},
    {16,4,0,0},
    {4,20,25,10},
    {9,0,0,0},
    {6,7,3,10}
};

int* get_candolist()//size=0~26,rawfood[0]=1，rawfood为int[51][3],有无，x,y
{
    int candolist[51]; //20-25为中间产物，26番茄炒蛋，27-40为成品菜，41-47为香锅，50位黑暗料理。
    for (int i = 0; i <= 50; i++)
    {
        candolist[i] = 0;
    }
    ///中间产物
    for (int i = 20; i <= 40; i++)
    {
        int i1 = i - 20;//菜肴编号-20用于检索howtodolist的行,rawfood[0][0]=1
        candolist[i] = rawfood[howtodolist[i1][0]][0] && rawfood[howtodolist[i1][1]][0] &&
            rawfood[howtodolist[i1][2]][0] && rawfood[howtodolist[i1][3]][0];
    }
    cout << "cando list generate finished!" << endl;
    return candolist;
}

int whichfood(int* candolist) {
    list<DishType> tk = task_list;
    int sum = 0;
    for (int i = 26; i <= 40; i++)//先看能不能制作成品菜
    {
        sum += candolist[i];
    }
    cout << "sum=" << sum << endl;
    if (sum != 0) {
        //优先制作任务列表里有的菜
        for (list<DishType>::iterator i = tk.end(); i != tk.begin(); i--)
        {
            cout << " tasklist :" << *i;
            if (*i<=50 && *i>=1 && candolist[*i] == 1) {
                cout << "make food in task list!" << endl;
                return *i;
            }
        }

        //如果都不在任务列表里，当你懒得考虑做什么菜，就随机一下,先看到哪个做哪个。
        vector<int> random;
        for (int i = 26; i <= 40; i++)
        {
            random.push_back(i);
        }
        random_shuffle(random.begin(), random.end());
        for (vector<int>::iterator i = random.begin(); i != random.end(); i++)
        {
            if (candolist[*i] == 1) return *i;
        }

    }
    else //一个成品菜都做不了，看看能不能合成中间产物,如果中间产物已经有了就算了不多做了。
    {
        if (candolist[20] == 1 && rawfood[20][0] != 1) return 20;//面粉》米饭》面条》番茄酱》奶油》面包
        if (candolist[23] == 1 && rawfood[23][0] != 1) return 23;
        if (candolist[24] == 1 && rawfood[24][0] != 1) return 24;
        if (candolist[21] == 1 && rawfood[21][0] != 1) return 21;
        if (candolist[25] == 1 && rawfood[25][0] != 1) return 25;
        if (candolist[22] == 1 && rawfood[21][0] != 1) return 22;
        return 0;//什么都做不了，return0
    }
    return 0;
}
//const char dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };

double angle_abs(Point &dest)//啊 是弧度
{
    double x = dest.x+0.5 - PlayerInfo.position.x;
    double y = dest.y+0.5 - PlayerInfo.position.y;
    cout << "destx=" << dest.x << ", posx=" << PlayerInfo.position.x << endl;
    cout << "desty=" << dest.y << ", posy=" << PlayerInfo.position.y << endl;
    if (x == 0) { x += 0.00000001; }
    if (y == 0) { y += 0.00000001; }//不知道=0会不会崩，偏一点点吧。
    return atan2(y,x);//atan2返回弧度
}
void put_dest(Point &dest,bool isdish)//计算从当前位置到目标位置，需要的角度和距离,目标x+0.5,y+0.5才是中心点
{
    double angle=angle_abs(dest);
    double dis = calcdis(Point(PlayerInfo.position.x, PlayerInfo.position.y), dest);
    cout << "put: dis=" << dis << "   angle:" << angle << endl;
    put(dis, angle, isdish);
    this_thread::sleep_for(time_50ms);
    return;
}
void move_allfood_to_right() {
    //先把灶台里所有的东西都丢到左边,并且修改刚刚的坐标到左边一格
    cout << "move all food to right" << endl;
    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);//这是灶台的坐标
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    char c = dir_4[cooklabel[label][3]];//朝向
    gotodest(Point(cooklabel[label][0], cooklabel[label][1]));
    move_dir(c, 1);//调整朝向
    list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        cout << "Obj TYPE :" << i->objType << " DISH:" << i->dish << endl;
        if (i->objType == Dish && i->dish != 0)
        {
            pick(FALSE, Dish, i->dish);
            this_thread::sleep_for(time_50ms);
            put(1, PI, TRUE);//扔到左边，修改坐标为本人左一格。
            rawfood[i->dish][1] = cooklabel[label][0] - 1;
            rawfood[i->dish][2] = cooklabel[label][1];
            this_thread::sleep_for(time_50ms);
        }
    }

}
int makefood(int food)//传入目标的编号
{

    if (food == 0)return 0 ;//如果收到的是0，那就啥都做不了，接着找食材吧
    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    char c = dir_4[cooklabel[label][3]];//朝向
    int row = food - 20;//行号
    move_allfood_to_right();
    //首先查找howtodolist[21][4]={{需要的食材编号,最多四种，多的置0}}第0行对应20面粉。
    cout << "start to pick food to cook" << endl;
    for (int i = 0; i <= 3; i++)
    {
        int destraw = howtodolist[row][i];//制作food需要食材的编号
        if (destraw == 0)break;//如果遇到0，说明后面没有需要的食材了。
        Point foodpos(rawfood[destraw][1], rawfood[destraw][2]);//查看食材所在位置
        gotodest(foodpos);
        int movex =sgn( (int)foodpos.x - (int)PlayerInfo.position.x);
        int movey =sgn( (int)foodpos.y - (int)PlayerInfo.position.y);
        char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
        if (fooddir == 's')//如果就在自己脚下
        {
            pick(TRUE, Dish, destraw);
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish == 0)return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
            put_dest(Point(cookx, cooky), TRUE);//这里+0.5没用，因为point是整数。

        }
        else
        {
            move_dir(fooddir,1);//调整朝向
            cout << "move : "<<fooddir << endl;
            pick(FALSE, Dish, destraw);
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish == 0)
            {
                cout << "pick filed!" << endl;
                return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
            }
            put_dest(Point(cookx+0.5, cooky+0.5), TRUE);
        }
        cout << "picked :" << destraw << endl;
    }
    gotodest(Point(cooklabel[label][0], cooklabel[label][1]));//再回到灶台跟前
    move_dir(c, 1);//调整朝向

    list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType == Dish) {
            int isneed = 0;//isneed=0表示不需要这个dish
            if (i->dish != 0) {
                for (int j = 0; j <= 4; j++)
                {
                    int destraw = howtodolist[row][j];//制作food需要食材的编号
                    if (destraw == 0)break;//如果遇到0，说明后面没有需要的食材了。
                    if (i->dish == destraw) {//如果能够和某个需要的食材匹配，就不扔出去了。
                        isneed = 1;
                    }
                }
                if (isneed == 0) {//不需要就统一扔到左边一格
                    pick(FALSE, Dish, i->dish);
                    this_thread::sleep_for(time_50ms);
                    put(1, PI, TRUE);
                    this_thread::sleep_for(time_50ms);
                }
                isneed = 0;
            }
        }
    }
    use(0, 0, 0);//开始做菜
    cout << "*****start to cook:" << food << endl;
    this_thread::sleep_for(time_50ms);
    l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        cout << "block type:" << i->blockType << "  dish:" << i->dish << endl;
    }
    return 1;//如果完成循环 那就是做完了。
}

const static int DishInfo[22][4] =
{
{int(Protobuf::Flour), 0,10000,0 },//20=0
{int(Protobuf::Noodle), 0,10000,0 },
{int(Protobuf::Bread), 0,10000,0 },
{int(Protobuf::CookedRice), 0,10000,0 },
{int(Protobuf::Ketchup), 0,10000,0 },
{int(Protobuf::Cream), 0,10000,0 },//25
{int(Protobuf::TomatoFriedEgg), 50,10000,60000 },
{int(Protobuf::TomatoFriedEggNoodle), 100,15000,90000},
{int(Protobuf::BeefNoodle), 80,20000,90000 },
{int(Protobuf::OverRice), 90,20000,90000},
{int(Protobuf::YellowPheasant), 100,20000,90000 },//30
{int(Protobuf::Barbecue), 55,20000,90000 },
{int(Protobuf::FrenchFries), 60,15000,90000},
{int(Protobuf::PlumJuice), 50,10000,90000 },
{int(Protobuf::Hamburger), 110,20000,100000 },
{int(Protobuf::StrawberryIcecream), 60,10000,90000 },//35
{int(Protobuf::PopcornChicken), 60,15000,90000 },
{int(Protobuf::AgaricFriedEgg), 50,15000,90000 },
{int(Protobuf::Cake), 160,30000,120000 },
{int(Protobuf::SugarCoatedHaws), 20,10000,60000 },
{int(Protobuf::FruitSalad), 100,20000,120000 },//40=20
{int(Protobuf::SpicedPot), 0,60000,300000 },
};

int cooking[6] = { 0,0,0,0,0,0 };//{dish,灶台label,用时，起，止，保护}
int state=0;
void play()
{
    //goto food
    //默认视野是9
    //默认速度是5

    //state0搜索食材并运到某个灶台
    cout << "label when start:"<<label << endl;
    gotodest(findnearfood());
    move_dir('w',1);//统一设置为，向上是食物产生点，这一步调整朝向。
    while (1)//守株待兔
    {
        pick_dish_in_block(findnearfood());
        if (PlayerInfo.dish != 0)
        {
            cout << "finish" << endl;
            break;//真的捡到了吗？捡到了就break，否则继续守株待兔
        }
        cout << "wait" << endl;
    }
    this_thread::sleep_for(time_50ms);
    cout <<"dish in hand : "<<PlayerInfo.dish<<endl;
    //准备找最近的灶台
    int labelofcook = findnearcook();//先找最近的灶台
    if (label != 0) { labelofcook = label; }//如果设置了据点，去据点，否则去最近的灶台。
    gotodest(Point(cooklabel[labelofcook][0], cooklabel[labelofcook][1])); //{x,y,编号，朝向，label}
    int nextstate=throw_darkdish(labelofcook);//到达灶台后，先检查有无黑暗料理
    cout << "nextstate:" << nextstate << endl;
    switch (nextstate)
    {
        case 0://自己的灶台正在做，这时候应该继续找食材再回来看。 
            cout << "my cook is used" << endl;
            break;
        case 1://这个灶台有别人正在用，找第二近的灶台
            cout << "this cook is being used" << endl;
            break;
        case 2://用这个灶台做菜就好，不用慌
            break;
        case 3://准备提交菜肴，此时菜肴已经在手里了。
            cout << "food in hand to be submit =" << PlayerInfo.dish << endl;
            break;
    }


    int angle = angle_4[cooklabel[labelofcook][3]];//从编号获得角度，扔到灶台里
    cout << "cooklabel=" << labelofcook << ",angle=" << angle << endl;
    cout <<"angle="<< angle << endl;
    put(1, angle_4[cooklabel[labelofcook][3]], TRUE);
    label = labelofcook;
    this_thread::sleep_for(time_50ms);
    //先把手里的食材放下来，然后开始做菜
    cout << "generate raw food list" << endl;
    get_all_dish(labelofcook);
    cout << "generate raw food finished" << endl;
    int dish_make = whichfood(get_candolist());
    cout << "which to make" << dish_make << endl;

    if (makefood(dish_make) == 1)
    {
        cooklabel[labelofcook][4] = 1;//自己正在做菜呢
        cout << "now make:" << dish_make<<endl;
        cooking[0] = dish_make;
        cooking[1] = labelofcook;
        cooking[2] = DishInfo[dish_make][2];//time
        cooking[3] = getGameTime()-50;//start
        cooking[4] = cooking[3] + cooking[2];//finished time
        cooking[5] = cooking[3] + cooking[2] * 1.25;
        
    }

    //state1 有灶台正在烹饪




    gotodest(findsecondfood());//找第二近的食材点
    cout << "**dish in hand :" << PlayerInfo.dish << endl;
    move_dir('w');//统一设置为，向上是食物产生点，这一步调整朝向。
    while (1)
    {
        pick_dish_in_block(findnearfood());
        if (PlayerInfo.dish != 0)
        {
            cout << "finish" << endl;
            break;//真的捡到了吗？捡到了就break，否则继续守株待兔
        }
        cout << "wait" << endl;
    }
    this_thread::sleep_for(time_50ms);
    cout << "**dish in hand : " << PlayerInfo.dish << endl;

    labelofcook = findnearcook();//先找最近的灶台
    if (label != 0) { labelofcook = label; }//如果设置了据点，去据点，否则去最近的灶台。
    gotodest(Point(cooklabel[labelofcook][0], cooklabel[labelofcook][1])); //{x,y,编号，朝向，label}
    angle = angle_4[cooklabel[labelofcook][3]];
    cout << "cooklabel="<<labelofcook<<",angle=" << angle << endl;
    put(1, angle_4[cooklabel[labelofcook][3]], TRUE);
    label = labelofcook;
    cout << "label before finish:" << label << endl;
    this_thread::sleep_for(time_50ms);
    cout << "**dish in hand :" << PlayerInfo.dish << endl<<endl;
    


}
