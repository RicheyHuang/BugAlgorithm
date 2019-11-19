#include <iostream>
#include <map>
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
        x = INT_MAX;
        y = INT_MAX;
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

    void setRotationError(const double& rotation_error_rad_)
    {
        rotation_error_rad = rotation_error_rad_;
    }

    void setTranslationError(const int& translation_error_pix_)
    {
        translation_error_pix = translation_error_pix_;
    }

    void setSonarRange(const double& range)
    {
        sonar_range = range;
    }

    int robot_radius;
    Point2D center;
    Direction2D direction;

    double sonar_range;

    double rotation_error_rad;
    int translation_error_pix;
};

bool DetectCollision(cv::Mat& map, Robot& cleanbot)
{
    bool isCollided = false;

    cv::Point circle_center (cleanbot.center.x, cleanbot.center.y);
    cv::Size axes (cleanbot.robot_radius+1, cleanbot.robot_radius+1);

    std::vector<cv::Point> robot_contour;
    int angle = 0;
    int arc_start = 0;
    int arc_end = 360;
    int delta = 1;

    cv::ellipse2Poly(circle_center, axes, angle, arc_start, arc_end, delta, robot_contour);

    for(const auto& point : robot_contour)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0))
        {
            isCollided = true;
            return isCollided;
        }
    }
    return isCollided;
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

void MoveRobot(Robot& cleanbot, const double& translate_pix, cv::Mat& map)
{
//    const double mean = 0.0;
//    const double stddev = 1.0;
//    std::default_random_engine random_num_generator(std::random_device{}());
//    std::normal_distribution<double> white_noise(mean, stddev);
//    double actual_translate_pix = translate_pix + cleanbot.translation_error_pix*white_noise(random_num_generator);

    std::mt19937 random_num_generator(std::random_device{}());
    std::uniform_real_distribution<> uniform_distribution(-cleanbot.translation_error_pix, cleanbot.translation_error_pix);
    double actual_translate_pix = translate_pix + uniform_distribution(random_num_generator);
    Point2D end = Point2D(cleanbot.center.x+int(cleanbot.direction[0]*actual_translate_pix), cleanbot.center.y+int(cleanbot.direction[1]*actual_translate_pix));
    Point2D start = cleanbot.center;

    cv::LineIterator path(map, cv::Point(start.x, start.y), cv::Point(end.x, end.y));
    for(int i = 0; i < path.count-1; i++)
    {
//        if()
//        {
//            break;
//        }
//        else
//        {
            cleanbot.center = Point2D(path.pos().x, path.pos().y);

            map.at<cv::Vec3b>(cleanbot.center.y, cleanbot.center.x) = cv::Vec3b(255, 0, 255);
            cv::imshow("map", map);
            cv::waitKey(1);

            path++;
//        }
    }
}

double Sonar(cv::Mat& map, Robot& cleanbot, const Direction2D& detect_direction)
{
    auto obstacle_dist = DBL_MAX;
    Point2D detect_point;

    int sonar_range = 0;
    for(int i = 1; i <= cleanbot.sonar_range; i++)
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

void DetectWall(cv::Mat& map, Robot& cleanbot, double& left_readout, double& right_readout)
{
    Direction2D left = Eigen::Rotation2Dd(-M_PI/2)*cleanbot.direction;
    Direction2D right = Eigen::Rotation2Dd(M_PI/2)*cleanbot.direction;

    left_readout = Sonar(map, cleanbot, left);
    right_readout = Sonar(map, cleanbot, right);
}

void BugAlgorithm1(cv::Mat& map, Robot& cleanbot, const double& rad_delta, const int& pix_delta, const int& accum_distance)
{
    int travelled_dist = 0;
    double left_readout, right_readout;
    double old_left_readout, old_right_readout;

    while(travelled_dist <= accum_distance)
    {
       DetectWall(map, cleanbot, left_readout, right_readout);

       if(left_readout < right_readout)
       {
           old_left_readout = left_readout;

           while(old_left_readout == left_readout)
           {
               RotateRobot(cleanbot, rad_delta);
               DetectWall(map, cleanbot, left_readout, right_readout);
           }

           if(left_readout < old_left_readout)
           {
                while(left_readout <= old_left_readout)
                {
                    old_left_readout = left_readout;
                    RotateRobot(cleanbot, rad_delta);
                    DetectWall(map, cleanbot, left_readout, right_readout);
                }
                RotateRobot(cleanbot, -rad_delta);
           }
           else if(left_readout > old_left_readout)
           {
               old_left_readout = left_readout;
               RotateRobot(cleanbot, -rad_delta);
               DetectWall(map, cleanbot, left_readout, right_readout);

               while(left_readout <= old_left_readout)
               {
                   old_left_readout = left_readout;
                   RotateRobot(cleanbot, -rad_delta);
                   DetectWall(map, cleanbot, left_readout, right_readout);
               }
               RotateRobot(cleanbot, rad_delta);
           }

           MoveRobot(cleanbot, pix_delta, map);
           travelled_dist += pix_delta;
       }
       else
       {
           old_right_readout = right_readout;

           while(old_right_readout == right_readout)
           {
               RotateRobot(cleanbot, rad_delta);
               DetectWall(map, cleanbot, left_readout, right_readout);
           }

           if(right_readout < old_right_readout)
           {
               while(right_readout <= old_right_readout)
               {
                   old_right_readout = right_readout;
                   RotateRobot(cleanbot, rad_delta);
                   DetectWall(map, cleanbot, left_readout, right_readout);
               }
               RotateRobot(cleanbot, -rad_delta);
           }
           else if(right_readout > old_right_readout)
           {
               old_right_readout = right_readout;
               RotateRobot(cleanbot, -rad_delta);
               DetectWall(map, cleanbot, left_readout, right_readout);

               while(right_readout <= old_right_readout)
               {
                   old_right_readout = right_readout;
                   RotateRobot(cleanbot, -rad_delta);
                   DetectWall(map, cleanbot, left_readout, right_readout);
               }
               RotateRobot(cleanbot, rad_delta);
           }

           MoveRobot(cleanbot, pix_delta, map);
           travelled_dist += pix_delta;


       }
    }
}

void BugAlgorithm(cv::Mat& map, Robot& cleanbot, const double& rad_delta, const int& pix_delta, const int& accum_distance)
{
    int travelled_dist = 0;
    double left_readout, right_readout;
    std::multimap<double, double, std::less<>> readout2angle;
    std::vector<double> angles;

    while(travelled_dist <= accum_distance)
    {
        DetectWall(map, cleanbot, left_readout, right_readout);

        if(left_readout < right_readout)
        {
            double rad = rad_delta;
            while(rad < 2*M_PI)
            {
                RotateRobot(cleanbot, double(rad_delta));
                DetectWall(map, cleanbot, left_readout, right_readout);
                readout2angle.insert(std::make_pair(left_readout, double(rad)));
                rad += rad_delta;
            }
            rad = 2*M_PI-(rad-rad_delta);
            RotateRobot(cleanbot, rad);

            double min_readout = readout2angle.begin()->first;

            std::cout<<min_readout<<"  ";

            std::pair<std::multimap<double, double>::iterator, std::multimap<double, double>::iterator> lower_upper = readout2angle.equal_range(min_readout);
            for(auto it = lower_upper.first; it != lower_upper.second; it++)
            {
                angles.emplace_back(it->second);
            }
            std::sort(angles.begin(), angles.end());
            rad = *(angles.begin()+(angles.size()/2));

            std::cout<<rad/M_PI*180<<std::endl;


            RotateRobot(cleanbot, rad);
        }
        else
        {
            double rad = rad_delta;
            while(rad < 2*M_PI)
            {
                RotateRobot(cleanbot, double(rad_delta));
                DetectWall(map, cleanbot, left_readout, right_readout);
                readout2angle.insert(std::make_pair(right_readout, double(rad)));
                rad += rad_delta;
            }
            rad = 2*M_PI-(rad-rad_delta);
            RotateRobot(cleanbot, rad);

            double min_readout = readout2angle.begin()->first;

            std::cout<<min_readout<<"  ";

            std::pair<std::multimap<double, double>::iterator, std::multimap<double, double>::iterator> lower_upper = readout2angle.equal_range(min_readout);
            for(auto it = lower_upper.first; it != lower_upper.second; it++)
            {
                angles.emplace_back(it->second);
            }
            std::sort(angles.begin(), angles.end());
            rad = *(angles.begin()+(angles.size()/2));


            std::cout<<rad/M_PI*180<<std::endl;


            RotateRobot(cleanbot, rad);
        }
        MoveRobot(cleanbot, pix_delta, map);
        travelled_dist += pix_delta;
        angles.clear();
        readout2angle.clear();
    }

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

    cv::Mat3b map(cv::Size(801, 801));
    map.setTo(cv::Vec3b(255, 255, 255));
    cv::circle(map, cv::Point(400, 400), 200, cv::Scalar(0, 0, 0), -1);

    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", map);
    cv::waitKey(0);

    Robot cleanbot = Robot(10, Point2D(20,20), Direction2D({1,1}));
    cleanbot.setSonarRange(100);
    cv::LineIterator line(map, cv::Point(20, 20), cv::Point(400, 400));
    for(int i = 0; i < line.count-1; i++)
    {
        if(DetectCollision(map, cleanbot))
        {
            cv::imshow("map", map);
            cv::waitKey(0);
            break;
        }
        cleanbot.center = Point2D(line.pos().x, line.pos().y);
        map.at<cv::Vec3b>(line.pos()) = cv::Vec3b(255, 0, 255);
        line++;
        cv::imshow("map", map);
        cv::waitKey(1);
    }
    BugAlgorithm(map, cleanbot, M_PI/180, 4, 8000);
    cv::imshow("map", map);
    cv::waitKey(0);
    return 0;
}