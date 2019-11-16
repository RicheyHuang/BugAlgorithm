#include <iostream>
#include <algorithm>
#include <random>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

typedef Eigen::Vector2d Direction2D;
struct Point2D
{
    Point2D()
    {
        x = DBL_MAX;
        y = DBL_MAX;
    }
    Point2D(const int& x_, const int& y_):x(x_), y(y_){}
    int x;
    int y;
};
typedef std::vector<Point2D> Path;

class Robot
{
public:
    Robot(const int& robot_radius_, const Point2D& center_, const Direction2D& direction_):robot_radius(robot_radius_), center(center_), direction(direction_)
    {
        direction.normalize();
        rotation_error_rad = 0.0;
        translation_error_pix = 0;
    }

    void setRotationError(double rotation_error_rad_)
    {
        rotation_error_rad = rotation_error_rad_;
    }

    void setTranslationError(int translation_error_pix_)
    {
        translation_error_pix = translation_error_pix_;
    }

    int robot_radius;
    Point2D center;
    Direction2D direction;

    double rotation_error_rad;
    int translation_error_pix;
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
//    const double mean = 0.0;
//    const double stddev = 1.0;
//    std::default_random_engine random_num_generator(std::random_device{}());
//    std::normal_distribution<double> white_noise(mean, stddev);
//    double actual_rotate_rad = rotate_rad + (cleanbot.rotation_error_rad/180.0*M_PI)*white_noise(random_num_generator);

    std::mt19937 random_num_generator(std::random_device{}());
    std::uniform_real_distribution<> uniform_distribution(-cleanbot.rotation_error_rad, cleanbot.rotation_error_rad);
    double actual_rotate_rad = rotate_rad + uniform_distribution(random_num_generator);

    cleanbot.direction = Eigen::Rotation2Dd(actual_rotate_rad) * cleanbot.direction;
}

void MoveRobot(Robot& cleanbot, const double& translate_pix)
{
//    const double mean = 0.0;
//    const double stddev = 1.0;
//    std::default_random_engine random_num_generator(std::random_device{}());
//    std::normal_distribution<double> white_noise(mean, stddev);
//    double actual_translate_pix = translate_pix + cleanbot.translation_error_pix*white_noise(random_num_generator);

    std::mt19937 random_num_generator(std::random_device{}());
    std::uniform_real_distribution<> uniform_distribution(-cleanbot.translation_error_pix, cleanbot.translation_error_pix);
    double actual_translate_pix = translate_pix + uniform_distribution(random_num_generator);

    cleanbot.center = Point2D(cleanbot.center.x+int(cleanbot.direction[0]*actual_translate_pix), cleanbot.center.y+int(cleanbot.direction[1]*actual_translate_pix));
}

double Sonar(cv::Mat& map, Robot& cleanbot, const Direction2D& detect_direction, const double& detect_range)
{
    double obstacle_dist = DBL_MAX;
    Point2D detect_point;

    int sonar_range = 0;
    for(int i = 1; i <= detect_range; i++)
    {
        detect_point = Point2D(cleanbot.center.x+int(detect_direction[0]*(cleanbot.robot_radius+i)),
                               cleanbot.center.y+int(detect_direction[1]*(cleanbot.robot_radius+i)));
        if(map.at<cv::Vec3b>(detect_point.y, detect_point.x) != cv::Vec3b(0, 0, 0))
        {
            sonar_range++;
        }
        else
        {
            obstacle_dist = sonar_range;
            return obstacle_dist;
        }
    }
    return obstacle_dist;
}

void DetectWall(cv::Mat& map, Robot& cleanbot, const double& detect_range, double& left_readout, double& right_readout)
{
    Direction2D left = Eigen::Rotation2Dd(-M_PI/2)*cleanbot.direction;
    Direction2D right = Eigen::Rotation2Dd(M_PI/2)*cleanbot.direction;

    left_readout = Sonar(map, cleanbot, left, detect_range);
    right_readout = Sonar(map, cleanbot, right, detect_range);
}

void BugAlgorithm()
{
    /**
     1. DetectCollision == true
     2. 顺时针旋转一个单位角度，判断左边传感器与障碍物的距离，通过旋转使其读数最小，即为与障碍物相切
     3. 前进一个单位距离，继续判断左边传感器与障碍物的距离，如果读数变大，则逆时针旋转一个单位角度
     4. 重复步骤3，运动需要加扰动
     */
}

void RotateRobotTest(double rotation_rad) {
    int robot_radius = 50;
    Point2D robot_center = Point2D(250, 250);
    Direction2D robot_direction = {0, -1};
    Robot cleanbot = Robot(robot_radius, robot_center, robot_direction);

    double rotate_err = 3.0/180.0*M_PI;
    cleanbot.setRotationError(rotate_err);

    cv::Mat3b map = cv::Mat3b(cv::Size(501,501));
    map.setTo(cv::Vec3b(0,0,0));

    cv::line(map, cv::Point(250,250), cv::Point(250+int(robot_radius*cleanbot.direction[0]),250+int(robot_radius*cleanbot.direction[1])), cv::Scalar(255, 255, 255));
    RotateRobot(cleanbot, rotation_rad);
    cv::line(map, cv::Point(250,250), cv::Point(250+int(robot_radius*cleanbot.direction[0]),250+int(robot_radius*cleanbot.direction[1])), cv::Scalar(255, 255, 0));

    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", map);
    cv::waitKey(0);
}

int main() {
    RotateRobotTest(M_PI);

    return 0;
}