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

    int robot_radius;
    Point2D center;
    Direction2D direction;

    double rotation_error_rad;
    int translation_error_pix;
};

bool DetectLeftCollision(cv::Mat& map, Robot& cleanbot)
{
    cv::Point circle_center (cleanbot.center.x, cleanbot.center.y);
    cv::Size axes (cleanbot.robot_radius+1, cleanbot.robot_radius+1);

    Direction2D x_axis = {1, 0};
    double yaw = std::atan2(cleanbot.direction[1], cleanbot.direction[0]) - std::atan2(x_axis[1], x_axis[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    int angle = int(90.0 + yaw/M_PI*180.0);
    int delta = 1;

    std::vector<cv::Point> left_arc;
    int left_arc_start = 180 + int(std::acos(1-2/cleanbot.robot_radius)/M_PI*180);
    int left_arc_end = 240;
    bool isLeftCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, left_arc_start, left_arc_end, delta, left_arc);

    for(const auto& point : left_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isLeftCollided = true;
            break;
        }
    }

    // visualization
    cv::Mat canvas = map.clone();
    cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
    if(isLeftCollided)
    {
        for(const auto& point : left_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
    }
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));
    cv::imshow("map", canvas);
    cv::waitKey(100);

    return isLeftCollided;
}

bool DetectFrontCollision(cv::Mat& map, Robot& cleanbot)
{
    cv::Point circle_center (cleanbot.center.x, cleanbot.center.y);
    cv::Size axes (cleanbot.robot_radius+1, cleanbot.robot_radius+1);

    Direction2D x_axis = {1, 0};
    double yaw = std::atan2(cleanbot.direction[1], cleanbot.direction[0]) - std::atan2(x_axis[1], x_axis[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    int angle = int(90.0 + yaw/M_PI*180.0);
    int delta = 1;

    std::vector<cv::Point> front_arc;
    int front_arc_start = 241;
    int front_arc_end = 299;
    bool isFrontCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, front_arc_start, front_arc_end, delta, front_arc);

    for(const auto& point : front_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isFrontCollided = true;
            break;
        }
    }

    // visualization

    cv::Mat canvas = map.clone();
    cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));

    if(isFrontCollided)
    {
        for(const auto& point : front_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
    }

    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));
    cv::imshow("map", canvas);
    cv::waitKey(100);

    return isFrontCollided;
}

bool DetectRightCollision(cv::Mat& map, Robot& cleanbot)
{
    cv::Point circle_center (cleanbot.center.x, cleanbot.center.y);
    cv::Size axes (cleanbot.robot_radius+1, cleanbot.robot_radius+1);

    Direction2D x_axis = {1, 0};
    double yaw = std::atan2(cleanbot.direction[1], cleanbot.direction[0]) - std::atan2(x_axis[1], x_axis[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    int angle = int(90.0 + yaw/M_PI*180.0);
    int delta = 1;

    std::vector<cv::Point> right_arc;
    int right_arc_start = 300;
    int right_arc_end = 360 - int(std::acos(1-2/cleanbot.robot_radius)/M_PI*180);
    bool isRightCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, right_arc_start, right_arc_end, delta, right_arc);

    for(const auto& point : right_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isRightCollided = true;
            break;
        }
    }

    // visualization

    cv::Mat canvas = map.clone();
    cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
    if(isRightCollided)
    {
        for(const auto& point : right_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
    }
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));
    cv::imshow("map", canvas);
    cv::waitKey(100);

    return isRightCollided;
}

std::vector<bool> DetectCollisions(cv::Mat& map, Robot& cleanbot)
{
    cv::Point circle_center (cleanbot.center.x, cleanbot.center.y);
    cv::Size axes (cleanbot.robot_radius+1, cleanbot.robot_radius+1);

    Direction2D x_axis = {1, 0};
    double yaw = std::atan2(cleanbot.direction[1], cleanbot.direction[0]) - std::atan2(x_axis[1], x_axis[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    int angle = int(90.0 + yaw/M_PI*180.0);
    int delta = 1;

    std::vector<cv::Point> left_arc;
    int left_arc_start = 180 + 30;
    int left_arc_end = 240;
    bool isLeftCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, left_arc_start, left_arc_end, delta, left_arc);

    for(const auto& point : left_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isLeftCollided = true;
            break;
        }
    }

    std::vector<cv::Point> front_arc;
    int front_arc_start = 241;
    int front_arc_end = 299;
    bool isFrontCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, front_arc_start, front_arc_end, delta, front_arc);

    for(const auto& point : front_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isFrontCollided = true;
            break;
        }
    }

    std::vector<cv::Point> right_arc;
    int right_arc_start = 300;
    int right_arc_end = 360 - 30;
    bool isRightCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, right_arc_start, right_arc_end, delta, right_arc);

    for(const auto& point : right_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isRightCollided = true;
            break;
        }
    }

    std::vector<bool> isCollided = {isLeftCollided||isFrontCollided||isRightCollided, isLeftCollided, isFrontCollided, isRightCollided};

    // visualization
    if(isCollided[0])
    {
        cv::Mat canvas = map.clone();
        cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
        if(isCollided[1])
        {
            for(const auto& point : left_arc)
            {
                canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
            }
        }
        if(isCollided[2])
        {
            for(const auto& point : front_arc)
            {
                canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
            }
        }
        if(isCollided[3])
        {
            for(const auto& point : right_arc)
            {
                canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
            }
        }
        cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
                 cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
                 cv::Scalar(0, 255, 0));
        cv::imshow("map", canvas);
        cv::waitKey(100);
    }

    return isCollided;
}

void RotateRobot(Robot& cleanbot, const double& rotate_rad, cv::Mat& map)
{
    std::mt19937 random_num_generator(std::random_device{}());
    std::uniform_real_distribution<> uniform_distribution(-cleanbot.rotation_error_rad, cleanbot.rotation_error_rad);
    double actual_rotate_rad = rotate_rad + uniform_distribution(random_num_generator);

    cleanbot.direction = Eigen::Rotation2Dd(actual_rotate_rad) * cleanbot.direction;

    // visualization
    cv::Mat canvas = map.clone();
    cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
            cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
            cv::Scalar(0, 255, 0));
    cv::imshow("map", canvas);
    cv::waitKey(1);
}

bool MoveForwardRobot(Robot& cleanbot, const double& translate_pix, cv::Mat& map)
{
    if(DetectCollisions(map, cleanbot)[2])
    {
        return false;
    }
    else
    {
        std::mt19937 random_num_generator(std::random_device{}());
        std::uniform_real_distribution<> uniform_distribution(-cleanbot.translation_error_pix,
                                                              cleanbot.translation_error_pix);
        double actual_translate_pix = translate_pix + uniform_distribution(random_num_generator);
        Point2D end = Point2D(cleanbot.center.x + int(cleanbot.direction[0] * (actual_translate_pix + 5)),
                              cleanbot.center.y + int(cleanbot.direction[1] * (actual_translate_pix + 5)));
        Point2D start = cleanbot.center;

        cv::LineIterator path(map, cv::Point(start.x, start.y), cv::Point(end.x, end.y));
        for (int i = 1; i <= actual_translate_pix; i++)
        {
            cleanbot.center = Point2D(path.pos().x, path.pos().y);

            // visualization
            cv::Mat canvas = map.clone();
            cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
            cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
                     cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
                     cv::Scalar(0, 255, 0));
            cv::imshow("map", canvas);
            cv::waitKey(1);


            if(DetectCollisions(map, cleanbot)[2])
            {
                break;
            }
            else
            {
                path++;
            }
        }
        return true;
    }
}

bool MoveBackwardRobot(Robot& cleanbot, const double& translate_pix, cv::Mat& map)
{
    RotateRobot(cleanbot, M_PI, map);
    bool isSuccessful = MoveForwardRobot(cleanbot, translate_pix, map);
    return isSuccessful;
}

double Sonar(cv::Mat& map, Robot& cleanbot, const Direction2D& detect_direction)
{
    Point2D detect_point;

    int range = 0;
    for(int i = 1; i <= INT_MAX; i++)
    {
        detect_point = Point2D(cleanbot.center.x+int(detect_direction[0]*(cleanbot.robot_radius+i)),
                               cleanbot.center.y+int(detect_direction[1]*(cleanbot.robot_radius+i)));

        if(detect_point.x < 0 || detect_point.x >= map.cols || detect_point.y < 0 || detect_point.y >= map.rows)
        {
            return range;
        }
        else
        {
            if(map.at<cv::Vec3b>(detect_point.y, detect_point.x) != cv::Vec3b(0, 0, 0))
            {
                range++;
            }
            else
            {
                return range;
            }
        }
    }
}

void DetectWall(cv::Mat& map, Robot& cleanbot, double& left_readout, double& right_readout)
{
    Direction2D left = Eigen::Rotation2Dd(-M_PI/2)*cleanbot.direction;
    Direction2D right = Eigen::Rotation2Dd(M_PI/2)*cleanbot.direction;

    left_readout = Sonar(map, cleanbot, left);
    right_readout = Sonar(map, cleanbot, right);
}

double DetectFront(cv::Mat& map, Robot& cleanbot)
{
    return Sonar(map, cleanbot, cleanbot.direction);
}

void QuickBugAlgorithm(cv::Mat& map, Robot& cleanbot, const double& rad_delta, const int& pix_delta, const int& accum_distance)
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
               RotateRobot(cleanbot, rad_delta, map);
               DetectWall(map, cleanbot, left_readout, right_readout);
           }

           if(left_readout < old_left_readout)
           {
                while(left_readout <= old_left_readout)
                {
                    old_left_readout = left_readout;
                    RotateRobot(cleanbot, rad_delta, map);
                    DetectWall(map, cleanbot, left_readout, right_readout);
                }
                RotateRobot(cleanbot, -rad_delta, map);
           }
           else if(left_readout > old_left_readout)
           {
               old_left_readout = left_readout;
               RotateRobot(cleanbot, -rad_delta, map);
               DetectWall(map, cleanbot, left_readout, right_readout);

               while(left_readout <= old_left_readout)
               {
                   old_left_readout = left_readout;
                   RotateRobot(cleanbot, -rad_delta, map);
                   DetectWall(map, cleanbot, left_readout, right_readout);
               }
               RotateRobot(cleanbot, rad_delta, map);
           }

           MoveForwardRobot(cleanbot, pix_delta, map);
           travelled_dist += pix_delta;
       }
       else
       {
           old_right_readout = right_readout;

           while(old_right_readout == right_readout)
           {
               RotateRobot(cleanbot, rad_delta, map);
               DetectWall(map, cleanbot, left_readout, right_readout);
           }

           if(right_readout < old_right_readout)
           {
               while(right_readout <= old_right_readout)
               {
                   old_right_readout = right_readout;
                   RotateRobot(cleanbot, rad_delta, map);
                   DetectWall(map, cleanbot, left_readout, right_readout);
               }
               RotateRobot(cleanbot, -rad_delta, map);
           }
           else if(right_readout > old_right_readout)
           {
               old_right_readout = right_readout;
               RotateRobot(cleanbot, -rad_delta, map);
               DetectWall(map, cleanbot, left_readout, right_readout);

               while(right_readout <= old_right_readout)
               {
                   old_right_readout = right_readout;
                   RotateRobot(cleanbot, -rad_delta, map);
                   DetectWall(map, cleanbot, left_readout, right_readout);
               }
               RotateRobot(cleanbot, rad_delta, map);
           }

           MoveForwardRobot(cleanbot, pix_delta, map);
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

    DetectWall(map, cleanbot, left_readout, right_readout);

    if(left_readout < right_readout)
    {
        while(travelled_dist <= accum_distance)
        {


            double rad = rad_delta;
            while (rad < 2 * M_PI) {
                RotateRobot(cleanbot, double(rad_delta), map);
                DetectWall(map, cleanbot, left_readout, right_readout);
                readout2angle.insert(std::make_pair(left_readout, double(rad)));
                rad += rad_delta;
            }
            rad = 2 * M_PI - (rad - rad_delta);
            RotateRobot(cleanbot, rad, map);

            double min_readout = readout2angle.begin()->first;

            std::pair<std::multimap<double, double>::iterator, std::multimap<double, double>::iterator> lower_upper = readout2angle.equal_range(
                    min_readout);
            for (auto it = lower_upper.first; it != lower_upper.second; it++) {
                angles.emplace_back(it->second);
            }
            std::sort(angles.begin(), angles.end());

            auto it = angles.begin();
            rad = *it;

            while(it != angles.end())
            {
                RotateRobot(cleanbot, rad, map);
                if(!MoveForwardRobot(cleanbot, pix_delta, map))
                {
                    it++;
                    rad = *it - rad;
                    continue;
                }
                else
                {
                    break;
                }
            }

            if(MoveForwardRobot(cleanbot, pix_delta, map))
            {
                travelled_dist += pix_delta;
            }

            angles.clear();
            readout2angle.clear();
        }
    }
    else
    {
        while(travelled_dist <= accum_distance)
        {
            cv::waitKey(0);


            double rad = rad_delta;
            while (rad < 2 * M_PI) {
                RotateRobot(cleanbot, double(rad_delta), map);
                DetectWall(map, cleanbot, left_readout, right_readout);
                readout2angle.insert(std::make_pair(right_readout, double(rad)));
                rad += rad_delta;
            }
            rad = 2 * M_PI - (rad - rad_delta);
            RotateRobot(cleanbot, rad, map);

            double min_readout = readout2angle.begin()->first;

            std::pair<std::multimap<double, double>::iterator, std::multimap<double, double>::iterator> lower_upper = readout2angle.equal_range(
                    min_readout);
            for (auto it = lower_upper.first; it != lower_upper.second; it++) {
                angles.emplace_back(it->second);
            }
            std::sort(angles.begin(), angles.end());

            auto it = angles.begin();
            rad = *it;

            while(it != angles.end())
            {
                RotateRobot(cleanbot, rad, map);
                if(!MoveForwardRobot(cleanbot, pix_delta, map))
                {
                    it++;
                    rad = *it - rad;
                    continue;
                }
                else
                {
                    break;
                }
            }

            if(MoveForwardRobot(cleanbot, pix_delta, map))
            {
                travelled_dist += pix_delta;
            }

            angles.clear();
            readout2angle.clear();
        }
    }
}

int main() {

    cv::Mat3b map(cv::Size(801, 801));
//    map.setTo(cv::Vec3b(255, 255, 255));
//    cv::circle(map, cv::Point(400, 400), 200, cv::Scalar(0, 0, 0), -1);
//    cv::circle(map, cv::Point(300, 400), 150, cv::Scalar(255, 255, 255), -1);

    cv::Mat canvas;

//    std::vector<cv::Point> contour = {cv::Point(300, 300), cv::Point(300, 500), cv::Point(500, 500), cv::Point(500, 300)};
//    std::vector<std::vector<cv::Point>> contours = {contour};
//    cv::fillPoly(map, contours, cv::Scalar(0, 0, 0));

    map.setTo(cv::Vec3b(0, 0, 0));
    std::vector<cv::Point> contour = {cv::Point(10, 10), cv::Point(10, 790), cv::Point(790, 790), cv::Point(790, 10)};
    std::vector<std::vector<cv::Point>> contours = {contour};
    cv::fillPoly(map, contours, cv::Scalar(255, 255, 255));

    cv::namedWindow("map", cv::WINDOW_NORMAL);

    Robot cleanbot = Robot(10, Point2D(20,20), Direction2D({1,1}));
    cv::LineIterator line(map, cv::Point(20, 20), cv::Point(800, 800));

    for(int i = 0; i < line.count-1; i++)
    {
        if(DetectCollisions(map, cleanbot)[2])
        {
            break;
        }
        cleanbot.center = Point2D(line.pos().x, line.pos().y);

        canvas = map.clone();
        cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
        cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
                 cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
                 cv::Scalar(0, 255, 0));
        cv::imshow("map", canvas);
        cv::waitKey(1);

        line++;
    }
    BugAlgorithm(map, cleanbot, M_PI/180, 4, 15000);
    cv::imshow("map", map);
    cv::waitKey(0);
    return 0;
}

int test()
{
    cv::Mat3b map(cv::Size(801, 801));
    map.setTo(cv::Vec3b(255, 255, 255));

    Robot cleanbot = Robot(100, Point2D(750,750), Direction2D({1,1}));

    cv::Point circle_center (cleanbot.center.x, cleanbot.center.y);
    cv::Size axes (cleanbot.robot_radius+1, cleanbot.robot_radius+1);

    std::vector<cv::Point> robot_contour;

    Direction2D x_axis = {1, 0};
    double yaw = std::atan2(cleanbot.direction[1], cleanbot.direction[0]) - std::atan2(x_axis[1], x_axis[0]);

    if(yaw > M_PI)
    {
        yaw -= 2*M_PI;
    }
    if(yaw < (-M_PI))
    {
        yaw += 2*M_PI;
    }

    int angle = int(90.0 + yaw/M_PI*180.0);
    int arc_start = 180;
    int arc_end = 360;
    int delta = 1;

    cv::ellipse2Poly(circle_center, axes, angle, arc_start, arc_end, delta, robot_contour);

//    cv::line(map, cv::Point(cleanbot.center.x, cleanbot.center.y), cv::Point(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius, cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius), cv::Scalar(0, 0, 255));
//
//    for(const auto& point : robot_contour)
//    {
//        map.at<cv::Vec3b>(point) = cv::Vec3b(255, 0, 0);
//    }

    cv::namedWindow("map", cv::WINDOW_NORMAL);
    cv::imshow("map", map);
    cv::waitKey(0);
}