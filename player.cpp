#ifdef __linux__
#include <bits/stdc++.h>
#endif

#include "API.h"
#include "Constant.h"
#include "player.h"
#include <iostream>
#include "OS_related.h"
#include <vector>
#include <list>
#include <math.h>
#include <array>
#include <string>
#include <sstream>
#include <thread>
#include <bits/stdc++.h>
#include <cmath>

#define PI 3.1415926535
using namespace THUAI3;
Protobuf::Talent initTalent = Protobuf::Talent::Runner;//指定人物天赋。选手代码必须定义此变量，否则报错

const int kCost1 = 10; //直移一格消耗
const int kCost2 = 15; //斜移一格消耗
const char dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };

struct Point {
    int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
    int F, G, H; //F=G+H	
    Point* parent; //parent的坐标，这里没有用指针，从而简化代码
    Point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL)  //变量初始化	
    {
    }
};

int cooking[6] = { 0,0,0,0,0,0 };//{0dish,1灶台label,2用时，3起，4止，5保护}
int state = 0;
int label;//记录“据点”是第几个灶台
int oldlabel = 0;
int surround[25][2];//懒得考虑返回值了，直接全局变量

double calcdis(Point point, Point end);
void get_all_dish(int _label);
void get_surround();
void findbestdish();
chrono::milliseconds time_200ms(200);
chrono::milliseconds time_100ms(100);
chrono::milliseconds time_50ms(50);
chrono::milliseconds time_25ms(25);

vector<vector<int>> foodgen =// type x y,距离 还是食物生成点下方一格
{
{0,0,0,99999},
{Wheat,4,24,0},
{Rice,5,5,0},
{Tomato,7,41,0},
{Egg,25,5,0},
{Beef,31,41,0},
{Pork,42,40,0},
{Potato,43,6,0},
{Lettuce,43,25,0}
};
bool sort_by_0(const vector<int> a, const vector<int> b)//默认从小到大排序
{
    return a[0] < b[0];
}
bool sort_by_0_double(const vector<double> a, const vector<double> b)//默认从小到大排序
{
    return a[0] < b[0];
}


bool sort_by_1(const vector<double> a, const vector<double> b)//默认从小到大排序
{
    return a[1] < b[1];
}


bool sort_by_3(const vector<int> a, const vector<int> b)//默认从小到大排序
{
    return a[3] < b[3];
}

void get_foodgen_dis()
{
    Point self(PlayerInfo.position.x, PlayerInfo.position.y);
    for (int i = 1; i <= 8; i++)
    {
        Point ifood(foodgen[i][1], foodgen[i][2]);
        foodgen[i][3] = int(calcdis(self, ifood));
    }
    sort(foodgen.begin(), foodgen.end(), sort_by_0);//确保顺序
}

list<vector<double>> bestdish = {  };//编号，性价比

//=============通用方法和结构=============
//点坐标
struct dPoint {
    double x;
    double y;
    dPoint(double _x, double _y) : x(_x), y(_y) {}
    dPoint(const dPoint& xy) : x(xy.x), y(xy.y) {}
    dPoint(const Point& xy) : x(xy.x), y(xy.y) {}
    dPoint(const XYPosition& xy) : x(xy.x), y(xy.y) {}
};
//某种食材有多少个
struct StoragePerDish {
    DishType type;        //食材名
    int stepsOfProcessed = 0;//如果是1，说明只需要合成一次，以此类推。
    int cnt = 0;          //现在有多少个
    list<dPoint> posList;  //储存地点链表
    StoragePerDish(DishType _type, int _cnt) : type(_type), cnt(_cnt) {};
};
/*用于角色切换行动的自动机状态表*/
enum State {
    IDLE = 0,         //闲置
    ONTHEWAY = 1,     //在路上
    WAITRESPONSE = 2  //等待队友返回消息
};
//=============全局常量=============

//储存目前所有物品的持有数量和储存地点，允许使用下标访问
class Storage {
private:
    vector<StoragePerDish> mstorage;
public:
    list<dPoint> condimentList;
    list<StoragePerDish> getRecipe(DishType _goal) {
        //返回一个非空表，如果_goal是可制作的或者已有的；否则空表
        list<DishType> recipeList{ _goal };  //未经判断的列表
        list<StoragePerDish> resultList{};

        bool isStillLoop = true;
        int loopDepth = 0;
        while (isStillLoop) {
            loopDepth++;
            isStillLoop = false;
            for (auto i = recipeList.begin(); i != recipeList.end();) {
                if (getCnt(*i) != 0) {  //如果该食材是已经有的
                    StoragePerDish tStorage = getStorage(*i);
                    tStorage.stepsOfProcessed = loopDepth;
                    resultList.push_back(tStorage);
                    i = recipeList.erase(i);//这会使迭代器i指向下一个元素
                }
                else {  //递归查找是否已经有存货来合成i指向的物品
                    auto findResult = Constant::CookingTable.find(*i);  // i的合成表
                    if (findResult != Constant::CookingTable
                        .end()) {  // i是可合成的，即，i不是最低级原料
                        isStillLoop = true;
                        recipeList.erase(i);  //删除i并将i的合成表合并至recipelist
                        list<DishType> resultList2 = findResult->second;
                        recipeList.merge(resultList2);
                        //自动重置循环
                        break;
                    }
                    else {  // i是不可合成的,且经过第一层if，i是存货中没有的
                        return list<StoragePerDish>();//返回一个空表
                    }
                }
            }
        }
        return resultList;
    }

public:
    Storage() {
        for (int i = 0; i < int(DishSize3); i++)
            mstorage.push_back(StoragePerDish(DishType(i), 0));
    }

    void add(DishType _dishType, dPoint _pos) {
        if (_dishType == 0) return;
        mstorage[int(_dishType)].cnt += 1;
        mstorage[int(_dishType)].posList.push_back(dPoint(_pos));
    }
    void addcondiment(dPoint _pos) {//调料放31吧
        condimentList.push_back(_pos);
    }
    int getCnt(DishType _dishType) { return mstorage[int(_dishType)].cnt; }

    StoragePerDish getStorage(DishType _dishType) {
        return mstorage[int(_dishType)];
    }

    list<dPoint> getStoragePos(DishType _dishType) {
        return mstorage[int(_dishType)].posList;
    }

    list<DishType> getDeficient(DishType _goal) {
        //返回所缺少的材料的List
        list<DishType> recipeList{ _goal };  //未经判断的列表
        list<DishType> resultList{};

        bool isStillLoop = true;

        while (isStillLoop) {
            isStillLoop = false;
            for (auto i = recipeList.begin(); i != recipeList.end();) {
                if (getCnt(*i) != 0) {  //如果该食材是已经有的，就跳过这个食材
                    i = recipeList.erase(i);  //这会使迭代器i指向下一个元素
                }
                else {  //递归查找是否已经有存货来合成i指向的物品
                    auto findResult = Constant::CookingTable.find(*i);  // i的合成表
                    if (findResult != Constant::CookingTable
                        .end()) {  // i是可合成的，即，i不是最低级原料
                        isStillLoop = true;
                        recipeList.erase(i);  //删除i并将i的合成表合并至recipelist
                        list<DishType> resultList2 = findResult->second;
                        recipeList.merge(resultList2);
                        break;  //重置循环
                    }
                    else {  // i是不可合成的,且经过第一层if，i是存货中没有的
                        resultList.push_back(*i);
                        i = recipeList.erase(i);
                    }
                }
            }
        }
        return resultList;
    }
    void updatestorage()
    {
        condimentList.clear();
        mstorage.clear();
        for (int i = 0; i < int(DishSize3); i++)
            mstorage.push_back(StoragePerDish(DishType(i), 0));

        get_surround();
        for (int i = 0; i <= 24; i++)
        {
            list<Obj> l = MapInfo::get_mapcell(surround[i][0], surround[i][1]);
            for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
            {
                if (i->dish != 0)//
                {
                    add(i->dish, dPoint(i->position.x, i->position.y));
                }
                if (i->tool == Condiment)//调料放单独一个list吧
                {
                    addcondiment(dPoint(i->position.x, i->position.y));
                }
            }
        }
        cout << endl << "*****************list begin ********************" << endl;
        for (int j = 10; j <= 21; j++) {
            list<StoragePerDish> st = getRecipe(DishType(j));
            cout << "try make :" << j << endl;
            for (auto i : st)
            {
                cout << "Dish :" << i.type << " step:" << i.stepsOfProcessed << endl;
            }
        }
        cout << "***************list end *********************" << endl << endl;
    }
    void cout_storage()
    {
        for (auto& i = mstorage.begin(); i != mstorage.end(); i++)
        {
            cout << "Dish:" << i->type << " cnt:" << i->cnt << endl;
        }
    }
    int getStorageSize()
    {
        int count = 0;
        for (int i = 1; i <= 8; i++)
        {
            if (getCnt(DishType(i)) != 0)count++;
        }
        cout << "size = " << count << endl;
        return count;
    }
};

Storage mystorage;
Storage nullstorage;
////////////////////////////////////////////////////////////////

DishType getGoal(list<DishType> raws) {
    //返回可以制作的目标，如果不能找到，则返回DishEmpty
    raws.sort();//先把传入数组从小到大排序
    for (auto i = Constant::CookingTable.begin();
        i != Constant::CookingTable.end(); i++) {//遍历合成表，查询有无和传入参数相同的表
        if (raws.size() != i->second.size())
            continue;
        auto p = raws.begin();
        auto q = i->second.begin();
        while (true) {
            if (p == raws.end())
                return DishType(i->first);
            if (int(*p++) != int(*q++))
                break;
        }
    }
    return DishType::DishEmpty;
}

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
    std::vector<Point*> getSurroundPoints_only(const Point* point, bool isIgnoreCorner) const;
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
        auto surroundPoints = getSurroundPoints(curPoint, isIgnoreCorner);//only就是只能上下左右了
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
    cout << "start:" << PlayerInfo.position.x << "," << PlayerInfo.position.y <<"end with "<<x<<","<<y<< endl;
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
    cout << "generate list  ";
    int i = 1;
    for (p = path.begin();;)
    {
        Point* pre = *p;
        p++;
        int movex = (*p)->x - pre->x;//dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };
        int movey = (*p)->y - pre->y;
        char c = dir[movey + 1][movex + 1];
        cout << c << ' ';
        gotolist.push_back(c);//例如dir[2][1]=w,dir[0][2]=c
        if (i == path.size() - 1)break;
        i++;
    }
    cout <<endl<< "return gotolist" << endl;
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

std::vector<Point*> Astar::getSurroundPoints_only(const Point* point, bool isIgnoreCorner) const {
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


std::vector<Point*> Astar::getSurroundPoints(const Point* point, bool isIgnoreCorner) const {
    std::vector<Point*> surroundPoints;
    for (int x = point->x - 1; x <= point->x + 1; x++)
        for (int y = point->y - 1; y <= point->y + 1; y++)
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
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
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
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
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
    {1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1},
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1} };

Astar astar(init_mapinfo_block);

void move_dir(char c, int i) //重载一下，i=1时只移动50ms表示时间，主要用于修改朝向。
{
    cout << "move only 50 * " << i << endl;
    switch (c)
    {
    case 'd':move(Protobuf::Direction::Right, 50 * i);
        break;
    case 'e':move(Protobuf::Direction::RightUp, 50 * i);
        break;
    case 'w':move(Protobuf::Direction::Up, 50 * i);
        break;
    case 'q':move(Protobuf::Direction::LeftUp, 50 * i);
        break;
    case 'a':move(Protobuf::Direction::Left, 50 * i);
        break;
    case 'z':move(Protobuf::Direction::LeftDown, 50 * i);
        break;
    case 'x':move(Protobuf::Direction::Down, 50 * i);
        break;
    case 'c':move(Protobuf::Direction::RightDown, 50 * i);
        break;

    default:
        break;
    }
    this_thread::sleep_for(time_50ms);
    for (int k = 1; k < i; k++)
    {
        this_thread::sleep_for(time_50ms);
    }
}

void move_dir(char c) //按照c中表示的方向移动
{
    double speed = PlayerInfo.moveSpeed;
    if (speed == 4) {
        switch (c)
        {
        case 'd':move(Protobuf::Direction::Right, 250);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'e':move(Protobuf::Direction::RightUp, 350);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);

            break;
        case 'w':move(Protobuf::Direction::Up, 250);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'q':move(Protobuf::Direction::LeftUp, 350);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'a':move(Protobuf::Direction::Left, 250);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'z':move(Protobuf::Direction::LeftDown, 350);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'x':move(Protobuf::Direction::Down, 250);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'c':move(Protobuf::Direction::RightDown, 350);
            this_thread::sleep_for(time_200ms);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;

        default:
            break;
        }
    }
    else//speed==9,
    {
        int time = 100;//希望移动的期望是111，要求有0.25
        if (rand() % 5 == 0)time = 150;
        switch (c)
        {
        case 'd':move(Protobuf::Direction::Right, time);
            this_thread::sleep_for(time_100ms);
            if (time == 150) this_thread::sleep_for(time_50ms);
            break;
        case 'e':move(Protobuf::Direction::RightUp, 150);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'w':move(Protobuf::Direction::Up, time);
            this_thread::sleep_for(time_100ms);
            if (time == 150) this_thread::sleep_for(time_50ms);
            break;
        case 'q':move(Protobuf::Direction::LeftUp, 150);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'a':move(Protobuf::Direction::Left, time);
            this_thread::sleep_for(time_100ms);
            if (time == 150) this_thread::sleep_for(time_50ms);
            break;
        case 'z':move(Protobuf::Direction::LeftDown, 150);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;
        case 'x':move(Protobuf::Direction::Down, time);
            this_thread::sleep_for(time_100ms);
            if (time == 150) this_thread::sleep_for(time_50ms);

            break;
        case 'c':move(Protobuf::Direction::RightDown, 150);
            this_thread::sleep_for(time_100ms);
            this_thread::sleep_for(time_50ms);
            break;

        default:
            break;
        }
    }
}

double calcdis(Point point, Point end) {
    //用简单的欧几里得距离计算H，这个H的计算是关键，还有很多算法，没深入研究^_^	
    return sqrt((double)(end.x - point.x) * (double)(end.x - point.x) + (double)(end.y - point.y) * (double)(end.y - point.y));
}

////////////////////////////////////////////////////////////////////////////////

Point findnearfood()//找最近的食物生成点
{
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    cout << "get dis" << endl;
    get_foodgen_dis();
    cout << "sort begin" << endl;
    sort(foodgen.begin(), foodgen.end(), sort_by_3);
    vector<int> destinfo = *foodgen.begin();
    Point nearest(destinfo[1], destinfo[2]);
    sort(foodgen.begin(), foodgen.end(), sort_by_0);
    return nearest;

}

Point findsecondfood()//找第二近的食物生成点
{
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    get_foodgen_dis();
    sort(foodgen.begin(), foodgen.end(), sort_by_3);
    vector<int> destinfo = foodgen[1];
    Point secondp(destinfo[1], destinfo[2]);
    sort(foodgen.begin(), foodgen.end(), sort_by_0);
    return secondp;
}

char dir_4[5] = { 0,'a','w','d','x' };//用这个数组存一下最后一步的方向
double angle_4[5] = { 0, PI,PI / 2,0,-PI / 2 };//用这个数组存一下最后扔的方向
int cooklabel[5][5] = { {}, {8,25,1,4,0}, {25,37,2,2,0}, {40,28,3,3,0}, {33,17,4,2,0} };
//第一行都是空着备用。
//【灶台边空地坐标xy,灶台编号，最后一步的方向,label(检查的时候方便吧大概)，在做菜吗】

Point findfarfood()//找第二近的食物生成点
{
    Point Point1(25, 4), Point2(42, 39), Point3(7, 40);//三个食物生成点下方一格，这样统一再往上走一步就好
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(Point1, Pos);
    double dis2 = calcdis(Point2, Pos);
    double dis3 = calcdis(Point3, Pos);
    if (dis1 >= dis2 && dis1 >= dis3) return Point1;
    if (dis2 >= dis1 && dis2 >= dis3) return Point2;
    if (dis3 >= dis1 && dis3 >= dis2) return Point3;
}

int findnearcook()//找最近的灶台
{
    Point Point1(8, 25), Point2(25, 37), Point3(40, 28), Point4(33, 17);//四个灶台
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
    if (c == 'd' || c == 'e' || c == 'c')return 1;
    if (c == 'a' || c == 'q' || c == 'z')return -1;
    else return 0;
}

int nexty(char c)
{
    if (c == 'w' || c == 'q' || c == 'e')return 1;
    if (c == 'x' || c == 'z' || c == 'c')return -1;
    else return 0;
}
Point getnear(Point dest)
{
    int x, y;
    Point self(PlayerInfo.position.x, PlayerInfo.position.y);
    vector<vector<double>> dis;//dis,x,y,
 //   cout << "change dest" << endl;
    if (astar.maze[dest.x + 1][dest.y] == 0) {
        x = dest.x + 1;
        y = dest.y;
        vector<double> push = { calcdis(self, Point(x, y)),double(x),double(y) };
        dis.push_back(push);
    }
    if (astar.maze[dest.x - 1][dest.y] == 0) {
        x = dest.x - 1;
        y = dest.y;
        vector<double> push = { calcdis(self, Point(x, y)),double(x),double(y) };
        dis.push_back(push);
    }
    if (astar.maze[dest.x][dest.y + 1] == 0) {
        x = dest.x;
        y = dest.y + 1;
        vector<double> push = { calcdis(self, Point(x, y)),double(x),double(y) };
        dis.push_back(push);
    }
    if (astar.maze[dest.x + 1][dest.y - 1] == 0) {
        x = dest.x;
        y = dest.y - 1;
        vector<double> push = { calcdis(self, Point(x, y)),double(x),double(y) };
        dis.push_back(push);
    }
    sort(dis.begin(), dis.end(), sort_by_0_double);//按距离从小到大排序
    x = (int)(*dis.begin())[1];
    y = (int)(*dis.begin())[2];
    //  cout << "change dest : " << x << "," << y <<"dis : "<<dis.front()[3]<<endl;
    return Point(x, y);
}

int gotodest(Point dest, int istimelimited = 0)//默认不限制，如果限制，把20000改成cooklabel[4]-5000,返回值为1表示成功
{
    list<char>::iterator lp;
    int x = dest.x;
    int y = dest.y;
    //用Getpath获得一个由字符组成的链表，字符代表移动方向，每一个是用200ms的时间走一格。
    cout << "speed" << PlayerInfo.moveSpeed << endl;
    if (astar.maze[dest.x][dest.y] == 1)//首先检测目标是否是障碍物，如果是，搜索周围四格中不是障碍物的点作为目标。
    {
        Point posnear = getnear(dest);
        x = posnear.x;
        y = posnear.y;
    }
    if ((int)PlayerInfo.position.x == x && (int)PlayerInfo.position.y == y)//如果已经在目的地，没必要移动。
    {
        cout << "no need move" << endl;
        return 1;
    }
    int timeinit = getGameTime();
    int time;
    while (1) {
        if (istimelimited == 0) {
            time = getGameTime() - timeinit;
            cout << "time" << time << endl;
            if (time >= 20000) {
                cout << "kia zhu le !!!" << endl;
                return 0;//这么久肯定是被卡住了，放弃吧。
            }
        }
        else
        {
            if (cooking[4] < getGameTime() + 5000) {
                cout << "xian hui qu ba" << endl;
                return 0;//正在做菜呢，先回去不然糊了
            }
        }
        for (auto& lp : astar.GetPath(x, y, false))
        {
            double pos_prex = PlayerInfo.position.x;
            double pos_prey = PlayerInfo.position.y;
            double pos_nextx = pos_prex + nextx(lp);
            double pos_nexty = pos_prey + nexty(lp);
            Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
            double disc;//和label灶台的距离，如果太近，不捡调料
            if (label == 0)
            {
                disc = 9999;
            }
            else
            {
                int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);//这是灶台的坐标
                int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
                Point cook(cookx, cooky);
                disc = calcdis(cook, Pos);//计算和灶台的距离，太近就不捡材料
                cout << "dis to cook = " << disc << endl;
            }      
            //list<Obj> l = MapInfo::get_mapcell((int)pos_nextx, (int)pos_nexty);//先检查下一个格子有没有能捡起来的或者trigger
            //trigger不检查了 就检查脚下有没有东西吧
            list<Obj> l = MapInfo::get_mapcell((int)pos_prex, (int)pos_prey);
            for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
            {
                if (i->objType == Tool) {
                    int t = i->tool;
                    cout << "tool" << t << endl;
                    switch (t)
                    {
                    case StrengthBuff://3
                    case TeleScope://4
                    case BreastPlate://7 护心镜
                        pick(TRUE, Tool, t);
                        this_thread::sleep_for(time_50ms);
                        if (PlayerInfo.tool == t) {
                            use(1, 0, 0);
                            this_thread::sleep_for(time_50ms);
                        }
                        cout << "use tool " << t << endl;
                        break;
                    case SpaceGate://8 传送门
                        pick(TRUE, Tool, t);
                        this_thread::sleep_for(time_50ms);
                        if (PlayerInfo.tool == t) {
                            use(1, x - (int)PlayerInfo.position.x, y - (int)PlayerInfo.position.y == y);//相对位移
                            this_thread::sleep_for(time_50ms);
                            cout << "use tool " << t << endl;
                            if (PlayerInfo.position.x == x && PlayerInfo.position.y == y) return 1;
                        }
                        break;
                    case Condiment://调料
                        //对于灶台周围区域，只在做菜的时候捡起condiment，走路不捡

                        if (label != 0 && disc >= 3)
                        {
                            pick(TRUE, Tool, t);
                            this_thread::sleep_for(time_50ms);
                            cout << "pick condiment " << t << endl;
                        }
                        else
                        {
                            cout << "near cook not pick condiment!" << endl;
                        }
                        break;
                    default:break;
                    }
                    //if (i->objType == 4) cout << "trigger" << i->trigger << endl;
                }
            }
            move_dir(lp);//处理完刚刚的问题再移动
            //cout << "form" << PlayerInfo.position.x << "  " << PlayerInfo.position.y << "  move:" << lp << endl;
            if (PlayerInfo.position.x == pos_prex && PlayerInfo.position.y == pos_prey) {
                cout << "error!" << PlayerInfo.position.x << "," << PlayerInfo.position.y << endl;
                //char dir_4[5] = { 0,'a','w','d','x' };//用这个数组存一下最后一步的方向
                int rand1 = rand() % 4 + 1;//1,2,3,4
                int rand2 = rand() % 2 - 1;//-1,0,1  0~5
                int add12 = rand1 + rand2;
                if (add12 == 0)add12 = 4;
                if (add12 == 5) add12 = 1;
                cout << "random move" << dir_4[rand1] << dir_4[add12] << endl;;
                move_dir(dir_4[rand1]);
                move_dir(dir_4[add12]);
                break;//break for 循环
            }//如果移动失败了，随机向某个方向移动一格，无论是否成功，都break循环，重新生成Getpath
            //cout << THUAI3::getGameTime() << endl;
        }
        if ((int)PlayerInfo.position.x == (int)x && (int)PlayerInfo.position.y == (int)y)
        {
            cout << "move finish!" << endl;
            return 1;
        }
        //直到到达目的地再break循环。
    }
    return 1;
}


void smallmove(double x, double y)//仅用于微小的移动，通过50ms=0.25个单位来实现,即差距乘4，四舍五入之后移动50ms
{
    double posx = PlayerInfo.position.x;
    double posy = PlayerInfo.position.y;
    double detx = x - posx;
    double dety = y - posy;
    double step = PlayerInfo.moveSpeed / 20;//每50ms移动step格。
    if (abs(detx) > 2 || abs(dety) > 2) return;
    if (detx >= step)
    {
        int count = round(detx / step);
        move_dir('d', count);//右移50
    }
    if (detx <= -step)
    {
        int count = round(abs(detx / step));
        move_dir('a', count);//左移50
    }
    if (dety >= step)
    {
        int count = round(dety / step);
        move_dir('w', count);//右移50
        count--;
    }
    if (dety <= -step)
    {
        int count = round(abs(detx / step));
        move_dir('x', count);//左移50
    }
}

//走到灶台后第一件事检查黑暗料理
int throw_darkdish(int _label)//检查灶台上有无黑暗料理，如果有，尝试拾取，拾取成功就扔掉，拾取失败就标记这个灶台不行，找下一个灶台
{
    int rand1 = rand() % 4 + 1;//1,2,3,4
    int rand2 = rand() % 2 - 1;//-1,0,1  0~5
    int add12 = rand1 + rand2;
    if (add12 == 0)add12 = 4;
    if (add12 == 5) add12 = 1;
    if (cooklabel[_label][4] == 1)return 0;//这个灶台是自己正在做的，不要扔黑暗料理
    //有时候自己做了菜，但是没有拿到手，这时候会把label变成0，但是这还是我的菜啊
    int x = cooklabel[_label][0] + nextx(dir_4[cooklabel[_label][3]]);
    int y = cooklabel[_label][1] + nexty(dir_4[cooklabel[_label][3]]);
    char c = dir_4[cooklabel[_label][3]];
    list<Obj> l = MapInfo::get_mapcell(x, y);
    string stop("s");//精细操作，要求对方停下来
    speakToFriend(stop);
    int mydish = PlayerInfo.dish;
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType == 3 && i->dish != 0)//如果发现灶台里面有菜
        {
            cout << "dish in cook!" << endl;
            put(0, PI, TRUE);//先把手里的东西放脚下           
            move_dir(c, 0);//确保朝向
            pick(FALSE, Block, 0);//尝试拾取
            this_thread::sleep_for(time_50ms);
            cout << "DISH IN BLOCK  " << i->dish << "  _label  " << _label << " label " << label << "  is my cook  " << cooklabel[label][4] << endl;
            if (PlayerInfo.dish == 0)//说明没有拿到dish,也就是这个灶台正在烹饪
            {
                //随便走两步，再拿一下试试
                cout << "random move" << dir_4[rand1] << dir_4[add12] << endl;;
                move_dir(dir_4[rand1]);
                move_dir(dir_4[add12]);
                gotodest(Point(cooklabel[_label][0], cooklabel[_label][1]));
                smallmove(cooklabel[_label][0] + 0.5, cooklabel[_label][1] + 0.5);
                move_dir(c, 1);//确保朝向
                pick(FALSE, Block, 0);//尝试拾取
                if (PlayerInfo.dish == 0) {
                    if (label == _label)label = 0;//如果这就是我标记的据点，那么取消标记。
                    pick(TRUE, Dish, mydish);
                    return 1;//继续跑第二近的灶台。
                }
            }
            if (PlayerInfo.dish >= OverCookedDish)//如果拿到了黑暗料理，扔出去
            {
                cout << "throw dark dish  " << PlayerInfo.dish << endl;
                put(2, 0, TRUE);//先往右扔两格,我下次看看最多能扔多远……
                //mystorage.add((DishType)mydish, dPoint(PlayerInfo.position.x, PlayerInfo.position.y));
                return 2;//用这个灶台做菜，不要慌
            }
            else //不是黑暗料理！捡到宝了！
            {
                //mystorage.add((DishType)mydish, dPoint(PlayerInfo.position.x, PlayerInfo.position.y));
                return 3;//准备提交食物，耶
            }
        }

    }
    return 2;//灶台里啥都没有，直接做菜。
}

int findallcook(int labelofcook)//从label开始转一圈圈,int=1表示找到了可以用的灶台，int=2表示拿到菜了。
{
    int flabel = labelofcook;
    int isfind = 0;
    while (isfind == 0)
    {
        flabel = labelofcook + 1;//2 3 4 5
        if (flabel == 5)flabel = 1;
        cout << "goto next label=" << flabel << endl;
        gotodest(Point(cooklabel[flabel][0], cooklabel[flabel][1])); //{x,y,编号，朝向，label}
        int nextstate = throw_darkdish(flabel);//到达灶台后，先检查有无黑暗料理
        cout << "label  " << label << "  is my cook  " << cooklabel[label][4] << endl;
        stringstream slabinfo;
        string labinfo;
        switch (nextstate)
        {
        case 0://这是我的据点？我正在做菜？bug吧？
            cout << "what wrong?" << endl;
            if (label != flabel) {
                label = flabel;
                /////
                labinfo.assign(slabinfo.str());
                slabinfo << 'l' << " " << label;
                cout << "send to team:" << slabinfo.str() << endl;
                speakToFriend(labinfo);
                /////
            }
            isfind = 1;
            break;
        case 1://继续跑第二近的灶台。
            cout << "next next label" << endl;
            break;
        case 2:
            if (label != flabel) {
                label = flabel;//用这个灶台好啦
                                /////
                labinfo.assign(slabinfo.str());
                slabinfo << 'l' << " " << label;
                cout << "send to team:" << slabinfo.str() << endl;
                speakToFriend(labinfo);
                /////
            }
            cout << "this is ok!" << endl;
            isfind = 1;
            return 1;
            break;
        case 3:
            cout << "get dish" << endl;
            state = 2;
            isfind = 1;
            return 2;
            break;
        }
    }
}


//1️⃣自身为中心共25格，顺序从远到近,从2，2开始顺时针绕回中心。
void get_surround()
{
    int x = PlayerInfo.position.x;
    int y = PlayerInfo.position.y;
    int k = 0;
    for (int i = 2; i >= -2; i--) {
        surround[k][0] = x + 2;
        surround[k][1] = y + i;
        k++;
    }
    for (int i = 1; i >= -2; i--)
    {
        surround[k][0] = x + i;
        surround[k][1] = y - 2;
        k++;
    }
    for (int i = -1; i <= 2; i++)
    {
        surround[k][0] = x - 2;
        surround[k][1] = y + i;
        k++;
    }
    for (int i = -1; i <= 1; i++)
    {
        surround[k][0] = x + i;
        surround[k][1] = y + 2;
        k++;
    }
    for (int i = 1; i >= -1; i--) {
        surround[k][0] = x + 1;
        surround[k][1] = y + i;
        k++;
    }
    surround[k][0] = x;
    surround[k][1] = y - 1;
    k++;
    surround[k][0] = x - 1;
    surround[k][1] = y - 1;
    k++;
    surround[k][0] = x - 1;
    surround[k][1] = y;
    k++;
    surround[k][0] = x - 1;
    surround[k][1] = y + 1;
    k++;
    surround[k][0] = x;
    surround[k][1] = y + 1;
    k++;
    surround[k][0] = x;
    surround[k][1] = y;
    k++;
}


int get_one_dish(int x, int y)//看看食物生成点有没有食物
{
    list<Obj> l = MapInfo::get_mapcell(x, y);//先扫一遍食物生成点，看看有没有食物，如果有返回0
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->objType == Block && i->dish != 0)
        {
            cout << "dish:" << i->dish << endl;
            return 0;//
        }
    }
    l = MapInfo::get_mapcell((int)PlayerInfo.position.x, (int)PlayerInfo.position.y);
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->objType == Dish && i->dish != 0)
        {
            cout << "dish:" << i->dish << endl;
            return i->dish;//再扫一遍脚下，如果有返回dish
        }
    }
    return -1;//如果啥都没有，返回-1
}

int pick_dish_in_block(Point food,int timelimit=0) {//Point 是食物生成点

    if (((int)PlayerInfo.position.x) != food.x || ((int)PlayerInfo.position.y) != food.y) {
        gotodest(Point(food.x, food.y));
        int movex = sgn((int)food.x - (int)PlayerInfo.position.x);
        int movey = sgn((int)food.y - (int)PlayerInfo.position.y);
        char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
        move_dir(fooddir, 1);//调整朝向
    }
    int dish = get_one_dish((int)food.x, (int)food.y);
    while (dish == -1)//如果dish=-1说明没有食物,就一直等着
    {
        this_thread::sleep_for(time_50ms);
        dish = get_one_dish((int)food.x, (int)food.y);
        cout << "wait for dish" << endl;
        if (((int)PlayerInfo.position.x) != food.x || ((int)PlayerInfo.position.y) != food.y) {
            gotodest(Point(food.x, food.y));
            int movex = sgn((int)food.x - (int)PlayerInfo.position.x);
            int movey = sgn((int)food.y - (int)PlayerInfo.position.y);
            char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
            move_dir(fooddir, 1);//调整朝向
        }
        if (timelimit)
        {
            if (cooking[4] < getGameTime() + 5000)
            {
                return 0;
                cout << "time limited! go back!" << endl;
            }
                
        }
    }
    if (dish == 0)
    {
        pick(FALSE, Block, 0);//是block的时候第三个随便输入,表示捡起block里的食材
        this_thread::sleep_for(time_50ms);
        cout << "pick dish in block finish" << endl;
        move_dir(dir_4[rand() % 4 + 1]);//随机走一下，防止卡住

    }
    else
    {
        pick(TRUE, Dish, dish);//捡起脚下的食材
        this_thread::sleep_for(time_50ms);
        cout << "pick dish on the ground finish" << endl;
        move_dir(dir_4[rand() % 4 + 1]);//随机走一下，防止卡住
    }
    return PlayerInfo.dish;
}

int xiangguo;

int whichfood()
{
    int which = 0;
    if (find(task_list.begin(), task_list.end(), SpicedPot) != task_list.end())
    {
        int sizeraw = mystorage.getStorageSize();
        if (sizeraw >= 3 && mystorage.condimentList.empty() != TRUE)
        {
            cout << "can make spice pot!" << endl;
            return SpicedPot;//如果有调料，而且食材够，可以做香锅。
        }
    }
    cout << "which food to make?" << endl;
    if (mystorage.getRecipe(Flour).empty() == FALSE && mystorage.getCnt(DishType(Flour)) == 0)
        which = Flour;
    if (mystorage.getRecipe(Noodle).empty() == FALSE && mystorage.getCnt(DishType(Noodle)) == 0)
        which = Noodle;
    if (mystorage.getRecipe(Ketchup).empty() == FALSE && mystorage.getCnt(DishType(Ketchup)) == 0)
        which = Ketchup;
    if (mystorage.getRecipe(Bread).empty() == FALSE && mystorage.getCnt(DishType(Bread)) == 0)
        which = Bread;

    //做性价比最高的菜
    findbestdish();//编号 性价比，已完成排序
    for (auto i : bestdish)
    {
        int dish = i[0];
        if (mystorage.getRecipe(DishType(dish)).empty() == FALSE && mystorage.getCnt(DishType(dish)) == 0)
        {
            cout << "make best food" << endl;
            which = dish;//优先制作性价比高的菜
        }
    }
    //最高优先级：任务列表里有的
    list<DishType> tk = task_list;
    list<StoragePerDish> tododish;
    for (list<DishType>::iterator i = tk.end(); i != tk.begin(); i--)
    {
        if (*i <= DarkDish && *i > 0 && mystorage.getRecipe(*i).empty() == FALSE && mystorage.getCnt(*i) == 0)
        {
            cout << "make food in task list!  Dish:" << *i << endl;
            which = *i;//优先制作任务列表里有的菜
        }
    }


    int maxstep = 0;
    //检查一下刚刚到底要做的是什么

    if (which == TomatoFriedEggNoodle)//如果要做的是西红柿鸡蛋面
    {
        int tomatoegg = mystorage.getCnt(DishType(TomatoFriedEgg)), noodle = mystorage.getCnt(DishType(Noodle)), flour = mystorage.getCnt(DishType(Flour));
        if (tomatoegg != 0 && noodle != 0)
        {
            return TomatoFriedEggNoodle;
        }
        if (tomatoegg == 0)
        {
            return TomatoFriedEgg;//如果没有西红柿炒蛋，那就做
        }
        if (noodle == 0)//有西红柿炒鸡蛋和面粉或小麦
        {
            if (flour == 0) return Flour;
            return Noodle;
        }
    }
    else {
        list<StoragePerDish> st = mystorage.getRecipe(DishType(which));
        for (auto i : st)
        {
            if (i.stepsOfProcessed > maxstep) maxstep = i.stepsOfProcessed;
        }
        if (maxstep == 2)return which;//一步完成
        else {
            list<DishType> raw;
            for (auto i : st)
            {
                if (i.stepsOfProcessed == maxstep)//看看步数等于最大步数时，能做哪些中间产物
                {
                    raw.push_back(i.type);
                }
            }
            which = getGoal(raw);
        }
    }
    return which;
}

//const char dir[3][3] = { {'z','x','c' },{'a','s','d'},{'q','w','e'} };

double angle_abs(Point dest)//啊 是弧度
{
    double x = dest.x + 0.5 - PlayerInfo.position.x;
    double y = dest.y + 0.5 - PlayerInfo.position.y;
    cout << "destx=" << dest.x << ", posx=" << PlayerInfo.position.x << endl;
    cout << "desty=" << dest.y << ", posy=" << PlayerInfo.position.y << endl;
    if (x == 0) { x += 0.00000001; }
    if (y == 0) { y += 0.00000001; }//不知道=0会不会崩，偏一点点吧。
    return atan2(y, x);//atan2返回弧度
}
void put_dest(Point dest, bool isdish)//计算从当前位置到目标位置，需要的角度和距离,目标x+0.5,y+0.5才是中心点
{
    double angle = angle_abs(dest);
    double dis = calcdis(Point(PlayerInfo.position.x, PlayerInfo.position.y), dest);
    cout << "put: dis=" << dis << "   angle:" << angle << endl;
    put(dis, angle, isdish);
    this_thread::sleep_for(time_50ms);
    return;
}

void move_allfood_to_left() {
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
            this_thread::sleep_for(time_50ms);
        }
    }

}

//捡调料，参数为1要扔到灶台，参数为0不扔
int getcondiment(int isthrow)
{
    if (mystorage.condimentList.empty() == TRUE)return 0;

    Point foodpos(mystorage.condimentList.begin()->x, mystorage.condimentList.begin()->y);//查看食材所在位置
    gotodest(foodpos);
    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
    int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
    char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向

    if (fooddir == 's')//如果就在自己脚下
    {
        pick(TRUE, Tool, Condiment);
        this_thread::sleep_for(time_50ms);
    }
    else
    {
        move_dir(fooddir, 0);//调整朝向
        pick(FALSE, Tool, Condiment);
    }

    this_thread::sleep_for(time_50ms);
    if (PlayerInfo.tool == 0)
    {
        cout << "pick condiment failed!" << endl;
        return 0;//这样就是没捡起来，返回0失败，直接重新找食材。

    }
    if (isthrow == 1)//如果要扔到灶台，那扔呗
    {
        put_dest(Point(cookx, cooky), FALSE);//这里+0.5没用，因为point是整数。
        cout << "put condiment to cook" << endl;
    }
    return 1;
}


int makefood(int food)//传入目标的编号 whichfood
{

    if (food == 0)return 0;//如果收到的是0，那就啥都做不了，接着找食材吧
    string stop("s");//精细操作，要求对方停下来
    speakToFriend(stop);

    int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
    int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
    char c = dir_4[cooklabel[label][3]];//朝向
    int row = food - 20;//行号
    move_allfood_to_left();

    this_thread::sleep_for(time_50ms);
    mystorage.updatestorage();
    if (food == SpicedPot)//把调料和三-五种原料丢进去。
    {
        cout << "make spicepot!!" << endl;
        int sizeraw = mystorage.getStorageSize();
        if (sizeraw > 5)sizeraw = 5;
        getcondiment(1);//把调料扔到灶台上
        get_foodgen_dis();
        sort(foodgen.begin(), foodgen.end(), sort_by_3);//根据距离从小到大排序
        int k = 1;
        for (int j = 1; j <= sizeraw && k <= 8; j++)//j指示已经放入了多少个食材，k指示第几近的食材。
        {
            int destraw = foodgen[k][0];
            k++;
            if (mystorage.getCnt((DishType)destraw) == 0)
            {
                j--;//j不增加
            }
            else
            {
                cout << "spice raw number :" << j << endl;
                dPoint i = *mystorage.getStoragePos((DishType)destraw).begin();
                Point foodpos(i.x, i.y);//查看食材所在位置
                cout << " try to get :" << destraw << " in pos " << foodpos.x << "," << foodpos.y << endl;
                gotodest(foodpos);
                smallmove(foodpos.x, foodpos.y);
                int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
                int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
                char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
                cout << "picked :" << destraw << endl;
                if (fooddir == 's')//如果就在自己脚下
                {
                    pick(TRUE, Dish, destraw);
                    this_thread::sleep_for(time_50ms);
                    if (PlayerInfo.dish == 0)return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
                    put_dest(Point(cookx, cooky), TRUE);//这里+0.5没用，因为point是整数。

                }
                else
                {
                    move_dir(fooddir, 0);//调整朝向
                    cout << "move : " << fooddir << endl;
                    pick(FALSE, Dish, destraw);
                    this_thread::sleep_for(time_50ms);
                    if (PlayerInfo.dish == 0)
                    {
                        cout << "pick raw food failed!  pos:" << PlayerInfo.position.x << " , " << PlayerInfo.position.y << endl;
                        return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
                    }
                    put_dest(Point(cookx, cooky), TRUE);
                }

            }
        }
        sort(foodgen.begin(), foodgen.end(), sort_by_0);
        cout << "back to cook" << endl;
        gotodest(Point(cooklabel[label][0], cooklabel[label][1]));//再回到灶台跟前
        move_dir(c, 1);//调整朝向
        list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
        int iscondiment = 0;
        for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
        {
            if (i->tool == Condiment) iscondiment = 1;
        }
        if (iscondiment == 0) return 0;
    }
    else
    {
        list<StoragePerDish> st = mystorage.getRecipe(DishType(food));
        //先检索最大的step,然后把和最大的step相同的step都丢到灶台里
        int maxstep = 0;
        for (auto i : st)
        {
            cout << "try make :" << food << "    Dish :" << i.type << " step:" << i.stepsOfProcessed << endl;
            if (i.stepsOfProcessed > maxstep) maxstep = i.stepsOfProcessed;
        }
        if (maxstep == 1)return 0;//已经有成品菜了，不做这个。
        cout << "start to pick food to cook" << endl;
        for (auto i : st)
        {
            if (i.stepsOfProcessed == maxstep)//如果步数等于最大步数，就丢到锅里
            {

                int destraw = i.type;
                Point foodpos(i.posList.begin()->x, i.posList.begin()->y);//查看食材所在位置
                cout << " try to get :" << destraw << " in pos " << foodpos.x << "," << foodpos.y << endl;
                gotodest(foodpos);
                smallmove(foodpos.x, foodpos.y);
                int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
                int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
                cout << "movex:" << movex << "  movey:" << movey << endl;
                char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
                if (fooddir == 's')//如果就在自己脚下
                {
                    pick(TRUE, Dish, destraw);
                    this_thread::sleep_for(time_50ms);
                    if (PlayerInfo.dish == 0)return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
                    put_dest(Point(cookx, cooky), TRUE);//这里+0.5没用，因为point是整数。
                    cout << "put finish " << endl;

                }
                else
                {
                    move_dir(fooddir, 0);//调整朝向
                    cout << "move : " << fooddir << endl;
                    pick(FALSE, Dish, destraw);
                    this_thread::sleep_for(time_50ms);
                    if (PlayerInfo.dish == 0)
                    {
                        cout << "pick raw food failed!  pos:" << PlayerInfo.position.x << " , " << PlayerInfo.position.y << endl;
                        return 0;//这样就是没捡起来，返回0失败，直接重新找食材。
                    }
                    put_dest(Point(cookx, cooky), TRUE);
                }
                cout << "picked :" << destraw << endl;
            }
        }

        //如果要做香锅，调用getcondiment(int isthrow=1)。
        cout << "back to cook" << endl;
        gotodest(Point(cooklabel[label][0], cooklabel[label][1]));//再回到灶台跟前
        move_dir(c, 0);//调整朝向

        list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
        for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
        {
            auto rawlist = Constant::CookingTable.find(food)->second;
            if (i->blockType == Dish) {
                // i的合成表
                if (find(rawlist.begin(), rawlist.end(), Dish) == rawlist.end())//没有找到
                {
                    pick(FALSE, Dish, i->dish);
                    this_thread::sleep_for(time_50ms);
                    put(1, PI, TRUE);
                    this_thread::sleep_for(time_50ms);
                }
            }
        }
    }
    ///////////////////////////////////////////////////////////////////////

    smallmove(cooklabel[label][0] + 0.5, cooklabel[label][1] + 0.5);
    move_dir(c, 1);//调整朝向
    use(0, 0, 0);//开始做菜
    this_thread::sleep_for(time_50ms);
    smallmove(cooklabel[label][0], cooklabel[label][1]);
    move_dir(c, 1);//调整朝向
    use(0, 0, 0);//多试一次呗
    cout << "*****start to cook:" << food << endl;
    this_thread::sleep_for(time_50ms);
    list<Obj> l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
    for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
    {
        if (i->blockType == 3 && i->dish == DarkDish) return 1;//在做了在做了
        cout << "error :: block type:" << i->blockType << "  dish:" << i->dish << endl;
    }
    return 0;//如果刚刚没有return1，那就是没做上。
}


Point findsave()//找最近的食物储藏
{
    Point save1(2, 34), save2(27, 47), save3(33, 2);//准备三个储藏点，放到最近的一个藏起来
    Point Pos(PlayerInfo.position.x, PlayerInfo.position.y);
    double dis1 = calcdis(save1, Pos);
    double dis2 = calcdis(save2, Pos);
    double dis3 = calcdis(save3, Pos);
    if (dis1 <= dis2 && dis1 <= dis3) return save1;
    if (dis2 <= dis1 && dis2 <= dis3) return save2;
    if (dis3 <= dis1 && dis3 <= dis2) return save3;
}

int commitTask() {
    //手上拿有成品菜式时调用此方法
    Point StoragePos();//储存成品的地点
    static vector<int[3]> finishedDish;  //这里放已经完成的菜品清单
    int flag = -1;                      // 1:去交菜品 2:放置菜品至储存点
    int randpoint = rand() % 2;
    Point destsubmit(26, 24 + randpoint);//随机去两个点之一交任务
    Point save = findsave();//准备最近的藏匿点，并且发消息告诉队友
    if (find(task_list.begin(), task_list.end(), PlayerInfo.dish) != task_list.end()) {
        //手上的菜品在任务清单里面
        gotodest(destsubmit);
        string stop("s");//精细操作，要求对方停下来
        speakToFriend(stop);
        move_dir('a', 1);
        while (PlayerInfo.dish != 0 && find(task_list.begin(), task_list.end(), PlayerInfo.dish) != task_list.end())
        {
            if (PlayerInfo.tool == Condiment)
            {
                use(1, 0, 0);//用调料提交
                this_thread::sleep_for(time_50ms);
            }
            else
            {
                use(0, 0, 0);
                this_thread::sleep_for(time_50ms);
            }
        }
        if (PlayerInfo.dish != 0)//手上还有菜，说明任务超时了，把菜放回原来的地方
        {
            gotodest(save);
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish != 0) {
                stringstream sdishinfo;
                sdishinfo << 'd' << " " << PlayerInfo.dish << " " << (int)save.x << " " << (int)save.y;
                cout << "send" << sdishinfo.str() << endl;
                speakToFriend(string(sdishinfo.str()));

                put(0, 0, TRUE);
                this_thread::sleep_for(time_50ms);
                if (PlayerInfo.tool == Condiment)
                {
                    put(0, 0, FALSE);
                    this_thread::sleep_for(time_50ms);
                }
            }
            return 0;
        }
        return 1;//返回1说明提交成功
    }
    else //不在任务清单里 先存起来
    {
        mystorage.updatestorage();
        if (mystorage.condimentList.empty() == FALSE)
        {
            getcondiment(0);//捡起来拿在手上。提交的时候可以用
        }
        //d dish x y
        stringstream sdishinfo;
        sdishinfo << 'd' << " " << PlayerInfo.dish << " " << (int)save.x << " " << (int)save.y;
        cout << "send" << sdishinfo.str() << endl;
        gotodest(save);
        this_thread::sleep_for(time_50ms);
        put(0, 0, TRUE);
        this_thread::sleep_for(time_50ms);
        speakToFriend(string(sdishinfo.str()));
        if (PlayerInfo.tool == Condiment)
        {
            put(0, 0, FALSE);
        }
        return 0;//返回0说明存起来了
    }
}

void findbestdish()
{
    bestdish.clear();//list<array<int,2>>
    get_foodgen_dis();//更新距离各食物生产点的距离
    for (int i = CookedRice; i <= Hamburger; i++)//14~21，计算每个的性价比。
    {
        double sumdis = 0;
        list<DishType> ls = nullstorage.getDeficient(DishType(i));
        for (auto dish : ls)
        {
            sumdis += foodgen[dish][3];//计算各个食物生产点的距离之和

        }
        auto findres = Constant::DishInfo.find(i);
        double r_value = sumdis / (findres->second.Score);
        //sort默认从小到大排序,最好是r_value最小，也就是距离/分数约小越好
        vector<double> thisdish = { double(i),r_value };//编号，性价比
        bestdish.push_back(thisdish);
    }
    bestdish.sort(sort_by_1);
}

int findbestfoodgen(int timelimited = 0)//传入cookedtime
{
    bestdish.clear();//list<array<int,2>>
    get_foodgen_dis();//更新距离各食物生产点的距离
    cout << "find best dish" << endl;
    if (timelimited != 0 && timelimited <= 60000)
    {
        cout << "return rand() with time limited" << endl;
        vector<int> destinfo;
        sort(foodgen.begin(), foodgen.end(), sort_by_3);//根据距离从小到大排序
        cout << "sort finish" << endl;
        if (timelimited <= 30000)
            destinfo = foodgen[rand() % 5+1];//  前五个里面随机一个吧
        if (timelimited <= 15000)
            destinfo = foodgen[rand() % 3+1];//  前三个里面随机一个吧,去掉最近的一个，容易崩。。
        int mydish = destinfo[0];
        sort(foodgen.begin(), foodgen.end(), sort_by_0);
        
        return mydish;
    }
    else {
        for (auto i : task_list)//用任务列表计算性价比
        {
            cout << "calculate value from tklist:"<<i << endl;
            if (i != SpicedPot)
            {
                double sumdis = 0;
                list<DishType> ls = mystorage.getDeficient(DishType(i));
                if (ls.empty() == FALSE) {
                    for (auto dish : ls)
                    {
                        sumdis += foodgen[dish][3];//计算各个食物生产点的距离之和

                    }
                    auto findres = Constant::DishInfo.find(i);
                    double r_value = sumdis / (findres->second.Score);
                    cout << "sumdis = "<<sumdis << "  r-value = " << r_value << endl;
                    //sort默认从小到大排序,最好是r_value最小，也就是距离/分数约小越好
                    vector<double> thisdish = { double(i),r_value };//编号，性价比
                    bestdish.push_back(thisdish);
                }
                else
                {
                    cout << "empty " <<i<< endl;
                }
            }
        }
        if (bestdish.empty())
        {
            cout << "all empty" << endl;
            return 0;
        }
        bestdish.sort(sort_by_1);
        int mybest = (*bestdish.begin())[0];//dish的编号
        cout << "mybest - " << mybest << endl;
        list<DishType> st = mystorage.getDeficient(DishType(mybest));
        if (st.empty() || mybest <= 0 || mybest >= DarkDish)
        {
            cout << "return rand()" << endl;
            return rand() % 8 + 1;
        }
        int mydish = *st.begin();
        cout << "mydish" << mydish << " size: " << st.size() << endl;
        return mydish;
    }

}

void mainswitch(int nextstate)
{
    int angle, dish_make;
    int labelofcook = label;
    cout << "nextstate:" << nextstate << endl;
    stringstream slabinfo;
    string labinfo;
    int randput = rand() % 4 + 1;
    switch (nextstate)
    {
    case 0://自己的灶台正在做，这时候应该继续找食材再回来看。 
        cout << "my cook is used" << endl;
        state = 1;
        break;
    case 1://这个灶台有别人正在用，找第二近的灶台
        cout << "this cook is being used" << endl;
        //如果返回2说明手里有菜可以提交了
        if (findallcook(labelofcook) == 1)//如果找到了可以用的灶台
        {
            //把case2的复制一遍
            cout << "now find another cook! " << label << endl;
            labelofcook = label;
            //随机放在周围四格，防止被一锅端
            //put(1, angle_4[cooklabel[labelofcook][3]], TRUE);这是原来的定点投放。
            cout << "random put:" << randput;
            put(1, angle_4[randput], TRUE);
            if (PlayerInfo.tool == Condiment)
            {
                this_thread::sleep_for(time_50ms);
                put(0, 0, FALSE);//调料扔到脚下
            }
            if (label != labelofcook) {
                label = labelofcook;
                /////
                labinfo.assign(slabinfo.str());
                slabinfo << 'l' << " " << label;
                cout << "send to team:" << slabinfo.str() << endl;
                speakToFriend(labinfo);
                /////
            }
            this_thread::sleep_for(time_50ms);
            //先把手里的食材放下来，然后开始做菜
            cout << "generate raw food list" << endl;
            mystorage.updatestorage();
            dish_make = whichfood();
            cout << "which to make" << dish_make << endl;
            if (makefood(dish_make) == 1)
            {
                cooklabel[labelofcook][4] = 1;//自己正在做菜呢
                cout << "now make:" << dish_make << endl;
                cooking[0] = dish_make;
                cooking[1] = labelofcook;
                auto findres = Constant::DishInfo.find(dish_make);
                cooking[2] = findres->second.CookTime;//time
                cooking[3] = getGameTime() - 50;//start
                cooking[4] = cooking[3] + cooking[2];//finished time
                cooking[5] = cooking[3] + cooking[2] * 1.25;
                cout << "cooking: ";
                for (int i = 0; i <= 5; i++)
                {
                    cout << " " << cooking[i];
                }
                cout << endl;
                state = 1;//进入【有灶台正在烹饪】的状态
            }
            else
            {
                state = 0;
            }
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish != 0)
            {
                put(0, 0, TRUE);
            }
        }

        break;
    case 2://用这个灶台做菜就好，不用慌
        angle = angle_4[cooklabel[labelofcook][3]];//从编号获得角度，扔到灶台里
        cout << "cooklabel=" << labelofcook << ",angle=" << angle << endl;
        cout << "angle=" << angle << endl;
        //随机放在周围四格，防止被一锅端
        //put(1, angle_4[cooklabel[labelofcook][3]], TRUE);这是原来的定点投放。
        cout << "random put:" << randput;
        put(1, angle_4[randput], TRUE);
        if (PlayerInfo.tool == Condiment)
        {
            this_thread::sleep_for(time_50ms);
            put(0, 0, FALSE);//调料扔到脚下
        }
        if (label != labelofcook) {
            label = labelofcook;
            /////
            labinfo.assign(slabinfo.str());
            slabinfo << 'l' << " " << label;
            cout << "send to team:" << slabinfo.str() << endl;
            speakToFriend(labinfo);
            /////
        }
        this_thread::sleep_for(time_50ms);
        //先把手里的食材放下来，然后开始做菜
        cout << "generate raw food list" << endl;
        mystorage.updatestorage();

        dish_make = whichfood();
        cout << "which to make" << dish_make << endl;
        if (makefood(dish_make) == 1)
        {
            cooklabel[labelofcook][4] = 1;//自己正在做菜呢
            cout << "now make:" << dish_make << endl;
            cooking[0] = dish_make;
            cooking[1] = labelofcook;
            auto findres = Constant::DishInfo.find(dish_make);
            cooking[2] = findres->second.CookTime;//time
            cooking[3] = getGameTime() - 50;//start
            cooking[4] = cooking[3] + cooking[2];//finished time
            cooking[5] = cooking[3] + cooking[2] * 1.25;
            cooklabel[labelofcook][4] = 1;
            cout << "cooking: ";
            for (int i = 0; i <= 5; i++)
            {
                cout << " " << cooking[i];
            }
            cout << endl;
            state = 1;//进入【有灶台正在烹饪】的状态
        }
        else
        {
            state = 0;
        }
        this_thread::sleep_for(time_50ms);
        if (PlayerInfo.dish != 0)
        {
            put(0, 0, TRUE);
        }
        break;
    case 3://准备提交菜肴，此时菜肴已经在手里了。
        cout << "food in hand to be submit =" << PlayerInfo.dish << endl;
        state = 2;
        break;
    }
}


int myround = 0;
void play()
{
    this_thread::sleep_for(time_50ms);
    cout << "begin!" << endl;
    cout << "myround : " << myround << endl;
    if (myround == 0)
    {
        cout << "round 0 " << endl;
        Point foodpos = findnearfood();
        gotodest(foodpos);
        int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
        int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
        char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
        move_dir(fooddir, 1);//调整朝向
        while (1)//守株待兔
        {
            pick_dish_in_block(foodpos);
            if (PlayerInfo.dish != 0)
            {
                cout << "finish" << endl;
                break;//真的捡到了吗？捡到了就break，否则继续守株待兔
            }
            cout << "pick failed" << endl;
        }
        this_thread::sleep_for(time_50ms);
        cout << "dish in hand : " << PlayerInfo.dish << endl;
        //准备找最近的灶台
        int labelofcook = findnearcook();//先找最近的灶台
        label = labelofcook;
        gotodest(Point(cooklabel[labelofcook][0], cooklabel[labelofcook][1])); //{x,y,编号，朝向，label}
        int nextstate = throw_darkdish(labelofcook);//到达灶台后，先检查有无黑暗料理
        mainswitch(nextstate);
    }
    myround += 1;
    if (state == 0) {
        cout << endl << "************state 1 begin **********" << endl;
        int bestfoodgen = findbestfoodgen();
        cout << "best food gen=" << bestfoodgen << endl;
        Point foodpos(foodgen[bestfoodgen][1], foodgen[bestfoodgen][2]);
        gotodest(foodpos);
        int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
        int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
        char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
        move_dir(fooddir, 1);//调整朝向
        while (1)//守株待兔
        {
            pick_dish_in_block(foodpos);
            if (PlayerInfo.dish != 0)
            {
                cout << "finish" << endl;
                break;//真的捡到了吗？捡到了就break，否则继续守株待兔
            }
            cout << "pick failed" << endl;
        }
        this_thread::sleep_for(time_50ms);
        cout << "dish in hand : " << PlayerInfo.dish << endl;
        //准备找最近的灶台
        int labelofcook = findnearcook();//先找最近的灶台
        if (label != 0) {
            cout << "label of cook=label in state1: " << label << endl;
            labelofcook = label;
        }//如果设置了据点，去据点，否则去最近的灶台。
        gotodest(Point(cooklabel[labelofcook][0], cooklabel[labelofcook][1])); //{x,y,编号，朝向，label}
        //不用确保朝向，在throw里有了
        int nextstate = throw_darkdish(labelofcook);//到达灶台后，先检查有无黑暗料理
        cout << "nextstate:" << nextstate << endl;
        mainswitch(nextstate);
    }
    //state1 有灶台正在烹饪
    if (state == 1)
    {
        cout << endl << "************state 1 begin **********" << endl;
        int cookx = cooklabel[label][0] + nextx(dir_4[cooklabel[label][3]]);
        int cooky = cooklabel[label][1] + nexty(dir_4[cooklabel[label][3]]);
        int ddl = cooking[3] + cooking[2] * 0.8;//在结束时间前0.8必须往回赶
        int labelofcook = label;
        while (1) {
            //正常搬运食材
            int bestfoodgen = findbestfoodgen(cooking[2]);//和之前一样，参数是有时间限制的
            Point foodpos(foodgen[bestfoodgen][1], foodgen[bestfoodgen][2]);
            int isdaoda = gotodest(foodpos, 1);//时间限制

            if (isdaoda == 0)
            {
                cout << "time limited!" << endl;
            }
            else
            {
                int movex = sgn((int)foodpos.x - (int)PlayerInfo.position.x);
                int movey = sgn((int)foodpos.y - (int)PlayerInfo.position.y);
                char fooddir = dir[movey + 1][movex + 1];//计算最后的朝向
                move_dir(fooddir, 1);//调整朝向
                while (1 && isdaoda != 0)//守株待兔
                {
                    if (getGameTime() > ddl) {
                        break;
                    }
                    pick_dish_in_block(findnearfood(),1);
                    if (PlayerInfo.dish != 0 || getGameTime() > ddl)
                    {
                        break;//捡到了，或者ddl到了，就回去。
                        cout << "wait" << endl;
                    }
                    this_thread::sleep_for(time_50ms);
                    cout << "dish in hand : " << PlayerInfo.dish << endl;
                }
            }

            gotodest(Point(cooklabel[labelofcook][0], cooklabel[labelofcook][1])); //{x,y,编号，朝向，label}
            //随机放在周围四格，防止被一锅端
            //put(1, angle_4[cooklabel[labelofcook][3]], TRUE);这是原来的定点投放。
            int randput = rand() % 4 + 1;
            cout << "random put:" << randput << endl;
            put(1, angle_4[randput], TRUE);
            if (PlayerInfo.tool == Condiment)
            {
                this_thread::sleep_for(time_50ms);
                put(0, 0, FALSE);//调料扔到脚下
            }
            cout << "left time:" << cooking[4] << "now " << getGameTime() << endl;
            if (cooking[4] < getGameTime() + 5000 || cooking[4] - getGameTime()>100 * 60 * 1000) {//
                break;//如果很快就要到ddl了，就留在灶台这里吧,时间太长就离谱，暂时不知道为啥
            }
            cout << "wait for finish" << endl;
        }
        move_dir(dir_4[cooklabel[label][3]], 1);
        int wait = 1;
        while (wait)//等待收菜
        {
            cout << "wait for finish near cook" << endl;
            list<Obj>l = MapInfo::get_mapcell(cookx, cooky);//看看灶台里有啥
            for (list<Obj>::iterator i = l.begin(); i != l.end(); i++)
            {
                cout << "block type:" << i->blockType << "  dish:" << i->dish << endl;
                if (i->dish != DarkDish && i->blockType == 3)
                {
                    cout << "do not wait " << getGameTime() << endl;
                    wait = 0;
                }
            }

            if (getGameTime() >= cooking[4]) {//finished time
                cout << "out of time" << endl;
                wait = 0;
                break;
            }
            this_thread::sleep_for(time_50ms);
        }
        cout << "now time:" << getGameTime() << endl;
        string stop("s");//精细操作，要求对方停下来
        speakToFriend(stop);
        this_thread::sleep_for(time_50ms);
        smallmove(cooklabel[label][0] + 0.5, cooklabel[label][1] + 0.5);
        move_dir(dir_4[cooklabel[label][3]], 1);
        pick(FALSE, Block, 0);
        state = 0;
        this_thread::sleep_for(time_50ms);
        cout << "get dish:" << PlayerInfo.dish << "  dest dish:" << cooking[0] << endl;
        if (PlayerInfo.dish == 0)
        {
            cout << "try again" << endl;
            move_dir(dir_4[cooklabel[label][3]], 1);
            pick(FALSE, Block, 0);
            this_thread::sleep_for(time_50ms);
            cout << "get dish:" << PlayerInfo.dish << "  dest dish:" << cooking[0] << endl;
        }
        if (PlayerInfo.dish == cooking[0])
        {
            cout << "success get dish" << endl;
            if (cooking[0] >= TomatoFriedEggNoodle || cooking[0] == CookedRice ||
                (cooking[0] == TomatoFriedEgg && find(task_list.begin(), task_list.end(), PlayerInfo.dish) != task_list.end()))
            {
                cout << "ready to submit" << endl;
                state = 2;
            }
            else//否则是中间产物，丢到左边一格，准备下一次烹饪。
            {
                cout << "this is mid chanwu" << endl;
                put(1, PI, TRUE);
                state = 0;
            }
        }
        else {
            if (PlayerInfo.dish == 0)
            {
                cout << "failed get dish" << endl;
                state = 0;
            }
            if (PlayerInfo.dish >= OverCookedDish)
            {
                cout << "Dark dish!" << endl;
                put(2, 0, TRUE);
                state = 0;
            }
        }
        if (state == 1)
        {
            state = 0;
        }
        cooking[0] = 0;
        cooking[1] = 0;
        cooking[2] = 0;
        cooking[3] = 0;
        cooking[4] = 0;
        cooking[5] = 0;
        cooklabel[label][4] = 0;
        if (state == 0)//如果刚刚的结果是state==0，先在灶台边上再看一眼能做的菜。
        {
            cout << "contiue cook!!" << endl;
            mystorage.updatestorage();
            int dish_make = whichfood();
            cout << "which to make" << dish_make << endl;
            if (makefood(dish_make) == 1)
            {
                cooklabel[labelofcook][4] = 1;//自己正在做菜呢
                cout << "now make:" << dish_make << endl;
                cooking[0] = dish_make;
                cooking[1] = labelofcook;
                auto findres = Constant::DishInfo.find(dish_make);
                cooking[2] = findres->second.CookTime;//time
                cooking[3] = getGameTime() - 50;//start
                cooking[4] = cooking[3] + cooking[2];//finished time
                cooking[5] = cooking[3] + cooking[2] * 1.25;
                cout << "cooking: ";
                for (int i = 0; i <= 5; i++)
                {
                    cout << " " << cooking[i];
                }
                cout << endl;
                state = 1;//进入【有灶台正在烹饪】的状态
            }
            else
            {
                state = 0;
            }
            this_thread::sleep_for(time_50ms);
            if (PlayerInfo.dish != 0)
            {
                put(0, 0, TRUE);
            }
        }
        //send不出来就离谱
        stringstream slabinfo2;
        slabinfo2 << 'l' << " " << label;
        string labinfo2(slabinfo2.str());
        cout << "send to team2:" << slabinfo2.str() << endl;
        speakToFriend(labinfo2);
    }

    //state=2 准备上交
    if (state == 2)
    {
        cout << endl << "************state 2 begin **********" << endl;
        int iscommited = commitTask();
        state = 0;
    }

}