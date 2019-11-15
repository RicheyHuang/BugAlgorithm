#include <iostream>
#include <algorithm>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>

typedef Eigen::Vector2d Direction2D;

struct Point2D
{
    Point2D(const int& x_, const int& y_):x(x_), y(y_){}
    int x;
    int y;
};

class Robot
{
public:
    Robot(const int& robot_radius_, const Point2D& center_, const Direction2D& direction_):robot_radius(robot_radius_), center(center_), direction(direction_){}
    int robot_radius;
    Point2D center;
    Direction2D direction;
};

bool DetectCollision(cv::Mat& map, Robot& cleanbot)
{
    Point2D curr_pos = cleanbot.center;
    int robot_radius = cleanbot.robot_radius;
    Direction2D curr_direction = cleanbot.direction;

    Point2D detect_point = Point2D(int(curr_pos.x+(robot_radius+1)*curr_direction[0]), int(curr_pos.y+(robot_radius+1)*curr_direction[1]));
    return map.at<cv::Vec3b>(detect_point.y, detect_point.x) == cv::Vec3b(0, 0, 0);
}

void RotateRobot(Robot& cleanbot, const double& rotate_rad)
{
    Point2D curr_pos = cleanbot.center;
    Direction2D direction = cleanbot.direction;

    int x_rotated = int(((direction[0] - curr_pos.x) * cos(rotate_rad)) - ((curr_pos.y - direction[1]) * sin(rotate_rad)) + curr_pos.x);
    int y_rotated = int(((curr_pos.y - direction[1]) * cos(rotate_rad)) - ((direction[0] - curr_pos.x) * sin(rotate_rad)) + curr_pos.y);
    direction = {x_rotated, y_rotated};
    direction.normalize();

    cleanbot.direction = direction;
}

void DetectWall(cv::Mat& map, Robot& cleanbot, double& left_readout, double& right_readout)
{

}

void BugAlgorithm()
{
    /**
     1. DetectCollision == true
     2. 顺时针旋转一个单位角度，判断左边传感器与障碍物的距离，通过旋转使其读数最小，即为与障碍物相切
     3. 前进一个单位距离，继续判断左边传感器与障碍物的距离，如果读数变大，则逆时针旋转一个单位角度
     4. 重复步骤3，
     */
}

int main() {

    return 0;
}