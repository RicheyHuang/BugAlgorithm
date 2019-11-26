#include <iostream>
#include <map>
#include <deque>
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

const int WALL_FOLLOWING_DISABLED = 0;
const int LEFT_WALL_FOLLOWING_ENABLED = 1;
const int RIGHT_WALL_FOLLOWING_ENABLED = 2;

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

    std::deque<double> left_readouts;
    std::deque<double> right_readouts;

    double rotation_error_rad;
    int translation_error_pix;
};

double Sonar(cv::Mat& map, const Point2D& source_pos, const double& source_radius, const Direction2D& detect_direction)
{
    Point2D detect_point;

    int range = 0;
    for(int i = 1; i <= INT_MAX; i++)
    {
        detect_point = Point2D(source_pos.x+int(detect_direction[0]*(source_radius+i)),
                               source_pos.y+int(detect_direction[1]*(source_radius+i)));

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

void DetectWall(cv::Mat& map, Robot& cleanbot)
{
    Direction2D left = Eigen::Rotation2Dd(-M_PI/2)*cleanbot.direction;
    Direction2D right = Eigen::Rotation2Dd(M_PI/2)*cleanbot.direction;

    if(cleanbot.left_readouts.size()>=2)
    {
        cleanbot.left_readouts.pop_front();
    }
    cleanbot.left_readouts.emplace_back(Sonar(map, cleanbot.center, cleanbot.robot_radius, left));


    if(cleanbot.right_readouts.size()>=2)
    {
        cleanbot.right_readouts.pop_front();
    }
    cleanbot.right_readouts.emplace_back(Sonar(map, cleanbot.center, cleanbot.robot_radius, right));
}

bool DetectCollision(cv::Mat& map, Robot& cleanbot)
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

    std::vector<cv::Point> robot_arc;
    int robot_arc_start = 180 + 20;
    int robot_arc_end = 360 - 20;
    bool isCollided = false;

    cv::ellipse2Poly(circle_center, axes, angle, robot_arc_start, robot_arc_end, delta, robot_arc);

    for(const auto& point : robot_arc)
    {
        if(map.at<cv::Vec3b>(point) == cv::Vec3b(0,0,0)
           || point.x < 0
           || point.x >= map.cols
           || point.y < 0
           || point.y >= map.rows)
        {
            isCollided = true;
            break;
        }
    }

    // visualization
    cv::Mat canvas = map.clone();
    cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));

    if(isCollided)
    {
        for(const auto& point : robot_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
        cv::imshow("map", canvas);
        cv::waitKey(50);
    }
    else
    {
        cv::imshow("map", canvas);
        cv::waitKey(1);
    }

    return isCollided;
}

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
    int left_arc_start = 180 + int(std::acos(1-3/cleanbot.robot_radius)/M_PI*180);
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
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));

    if(isLeftCollided)
    {
        for(const auto& point : left_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
        cv::imshow("map", canvas);
        cv::waitKey(500);
    }
    else
    {
        cv::imshow("map", canvas);
        cv::waitKey(1);
    }
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
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));

    if(isFrontCollided)
    {
        for(const auto& point : front_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
        cv::imshow("map", canvas);
        cv::waitKey(500);
    }
    else
    {
        cv::imshow("map", canvas);
        cv::waitKey(1);
    }

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
    int right_arc_end = 360 - int(std::acos(1-3/cleanbot.robot_radius)/M_PI*180);
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
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
             cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
             cv::Scalar(0, 255, 0));

    if(isRightCollided)
    {
        for(const auto& point : right_arc)
        {
            canvas.at<cv::Vec3b>(point) = cv::Vec3b(0, 0, 255);
        }
        cv::imshow("map", canvas);
        cv::waitKey(500);
    }
    else
    {
        cv::imshow("map", canvas);
        cv::waitKey(1);
    }

    return isRightCollided;
}

double RotateRobot(Robot& cleanbot, const double& rotate_rad, cv::Mat& map)
{
    std::mt19937 random_num_generator(std::random_device{}());
    std::uniform_real_distribution<> uniform_distribution(-cleanbot.rotation_error_rad, cleanbot.rotation_error_rad);
    double actual_rotate_rad = rotate_rad + uniform_distribution(random_num_generator);

    cleanbot.direction = Eigen::Rotation2Dd(actual_rotate_rad) * cleanbot.direction;

    DetectWall(map, cleanbot);

//    std::cout<<"right readout: "<<cleanbot.right_readouts.back()<<std::endl;

    // visualization
    cv::Mat canvas = map.clone();
    cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
    cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
            cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
            cv::Scalar(0, 255, 0));
    cv::imshow("map", canvas);
    cv::waitKey(1);

    return actual_rotate_rad;
}

int MoveForwardRobot(Robot& cleanbot, const double& translate_pix, cv::Mat& map, const int& wall_following=WALL_FOLLOWING_DISABLED)
{
    std::mt19937 random_num_generator(std::random_device{}());
    std::uniform_real_distribution<> uniform_distribution(-cleanbot.translation_error_pix,
                                                          cleanbot.translation_error_pix);
    double actual_translate_pix = translate_pix + uniform_distribution(random_num_generator);
    Point2D end = Point2D(cleanbot.center.x + int(cleanbot.direction[0] * (actual_translate_pix + 5)),
                          cleanbot.center.y + int(cleanbot.direction[1] * (actual_translate_pix + 5)));
    Point2D start = cleanbot.center;

    cv::LineIterator path(map, cv::Point(start.x, start.y), cv::Point(end.x, end.y));

    for (int i = 0; i <= actual_translate_pix; i++)
    {
        cleanbot.center = Point2D(path.pos().x, path.pos().y);
        DetectWall(map, cleanbot);

//        std::cout<<"right readout: "<<cleanbot.right_readouts.back()<<std::endl;

        // visualization
        cv::Mat canvas = map.clone();
        cv::circle(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y), cleanbot.robot_radius, cv::Scalar(255, 0, 0));
        cv::line(canvas, cv::Point(cleanbot.center.x, cleanbot.center.y),
                 cv::Point(int(cleanbot.center.x+cleanbot.direction[0]*cleanbot.robot_radius), int(cleanbot.center.y+cleanbot.direction[1]*cleanbot.robot_radius)),
                 cv::Scalar(0, 255, 0));
        cv::imshow("map", canvas);
        cv::waitKey(1);

        if(wall_following == WALL_FOLLOWING_DISABLED)
        {
            if(DetectCollision(map, cleanbot))
            {
                return i;
            }
            else
            {
                path++;
            }
        }
        else if(wall_following == LEFT_WALL_FOLLOWING_ENABLED)
        {
            // abs necessary?
            if(std::abs(cleanbot.left_readouts.back() - cleanbot.left_readouts.front()) > 1e-6)
            {
                return i;
            }
            else
            {
                path++;
            }
        }
        else if(wall_following == RIGHT_WALL_FOLLOWING_ENABLED)
        {
            // abs necessary?
            if(std::abs(cleanbot.right_readouts.back() - cleanbot.right_readouts.front()) > 1e-6)
            {
                return i;
            }
            else
            {
                path++;
            }
        }
        else
        {
            std::cout<<"invalid flags."<<std::endl;
            return 0;
        }
    }
    return int(actual_translate_pix);
}

int MoveBackwardRobot(Robot& cleanbot, const double& translate_pix, cv::Mat& map, const int& wall_following=WALL_FOLLOWING_DISABLED)
{
    RotateRobot(cleanbot, M_PI, map);
    double actual_translation = MoveForwardRobot(cleanbot, translate_pix, map, wall_following);
    return actual_translation;
}

void TOFCamera(cv::Mat& map, Robot& cleanbot)
{
}

void BugAlgorithm(cv::Mat& map, Robot& cleanbot, const double& rad_delta, const int& pix_delta, const int& accum_distance)
{
    int travelled_dist = 0;
    DetectWall(map, cleanbot);

    if(cleanbot.left_readouts.back() < cleanbot.right_readouts.back())
    {
        while(travelled_dist <= accum_distance)
        {
            while(DetectCollision(map, cleanbot))
            {
                RotateRobot(cleanbot, 5*rad_delta, map);
            }

            double accum_readout = 0.0;
            while (!DetectCollision(map, cleanbot))
            {
                double actual_pix = MoveForwardRobot(cleanbot, pix_delta, map, LEFT_WALL_FOLLOWING_ENABLED);
                travelled_dist += int(actual_pix);
                accum_readout += (cleanbot.left_readouts.back() - cleanbot.left_readouts.front());

                if(DetectCollision(map, cleanbot))
                {
                    break;
                }
                else if(accum_readout > 2*cleanbot.robot_radius)
                {
                    accum_readout = 0.0;
                    Direction2D direction_anchor = cleanbot.direction;

                    std::multimap<double, double, std::less<>> readout2angle;
                    std::vector<double> angles;

                    std::deque<double> readout_deltas(int(cleanbot.robot_radius), DBL_MAX);
                    double readout = cleanbot.left_readouts.back();
                    double readout_delta = 0.0;

                    while(std::accumulate(readout_deltas.begin(), readout_deltas.end(), 0.0) > cleanbot.robot_radius)
                    {
                        double rad = rad_delta;
                        while (rad < 2* M_PI)
                        {
                            RotateRobot(cleanbot, double(rad_delta), map);
                            readout2angle.insert(std::make_pair(cleanbot.left_readouts.back(), double(rad)));
                            rad += rad_delta;
                        }
                        rad = 2* M_PI - (rad - rad_delta);
                        RotateRobot(cleanbot, rad, map);

                        double min_readout = readout2angle.begin()->first;

                        std::pair<std::multimap<double, double>::iterator, std::multimap<double, double>::iterator> lower_upper = readout2angle.equal_range(
                                min_readout);
                        for (auto it = lower_upper.first; it != lower_upper.second; it++)
                        {
                            angles.emplace_back(it->second);
                        }
                        std::sort(angles.begin(), angles.end());

                        auto it = angles.begin();
                        rad = *it;

                        while (it != angles.end())
                        {
                            RotateRobot(cleanbot, rad, map);
                            if (DetectCollision(map, cleanbot))
                            {
                                it++;
                                rad = *it - rad;
                                continue;
                            }
                            else
                            {
                                readout_delta = cleanbot.left_readouts.back() - readout;
                                readout_deltas.pop_front();
                                readout_deltas.emplace_back(readout_delta);
                                readout = cleanbot.left_readouts.back();
                                break;
                            }
                        }

                        if(DetectCollision(map, cleanbot))
                        {
                            break;
                        }
                        else
                        {
                            actual_pix = MoveForwardRobot(cleanbot, pix_delta, map, LEFT_WALL_FOLLOWING_ENABLED);
                            travelled_dist += int(actual_pix);

                            readout2angle.clear();
                            angles.clear();
                        }
                    }
                }

                else if(cleanbot.left_readouts.back() > 0.0)
                {
                    RotateRobot(cleanbot, -2*rad_delta, map);
                }
            }
        }
    }
    else
    {
        while(travelled_dist <= accum_distance)
        {
            while(DetectCollision(map, cleanbot))
            {
                RotateRobot(cleanbot, -5*rad_delta, map);
            }

            double accum_readout = 0.0;
            while (!DetectCollision(map, cleanbot))
            {
                int actual_pix = MoveForwardRobot(cleanbot, pix_delta, map, RIGHT_WALL_FOLLOWING_ENABLED);
                travelled_dist += int(actual_pix);
                accum_readout += (cleanbot.right_readouts.back() - cleanbot.right_readouts.front());

                if(DetectCollision(map, cleanbot))
                {
                    break;
                }
                else if(accum_readout > 2*cleanbot.robot_radius)
                {
                    accum_readout = 0.0;
                    Direction2D direction_anchor = cleanbot.direction;

                    std::multimap<double, double, std::less<>> readout2angle;
                    std::vector<double> angles;

                    std::deque<double> readout_deltas(int(cleanbot.robot_radius), DBL_MAX);
                    double readout = cleanbot.right_readouts.back();
                    double readout_delta = 0.0;

                    while(std::accumulate(readout_deltas.begin(), readout_deltas.end(), 0.0) > cleanbot.robot_radius)
                    {
                        double rad = -rad_delta;
                        while (rad > -2 * M_PI) {
                            RotateRobot(cleanbot, double(-rad_delta), map);
                            readout2angle.insert(std::make_pair(cleanbot.right_readouts.back(), double(rad)));
                            rad -= rad_delta;
                        }
                        rad = -2 * M_PI - (rad + rad_delta);
                        RotateRobot(cleanbot, rad, map);

                        double min_readout = readout2angle.begin()->first;

                        std::pair<std::multimap<double, double>::iterator, std::multimap<double, double>::iterator> lower_upper = readout2angle.equal_range(
                                min_readout);
                        for (auto it = lower_upper.first; it != lower_upper.second; it++) {
                            angles.emplace_back(it->second);
                        }
                        std::sort(angles.begin(), angles.end(), std::greater<>());

                        auto it = angles.begin();
                        rad = *it;

                        while (it != angles.end()) {
                            RotateRobot(cleanbot, rad, map);
                            if (DetectCollision(map, cleanbot))
                            {
                                it++;
                                rad = *it - rad;
                                continue;
                            }
                            else
                            {
                                readout_delta = cleanbot.right_readouts.back() - readout;
                                readout_deltas.pop_front();
                                readout_deltas.emplace_back(readout_delta);
                                readout = cleanbot.right_readouts.back();
                                break;
                            }
                        }

                        if(DetectCollision(map, cleanbot))
                        {
                            break;
                        }
                        else
                        {
                            actual_pix = MoveForwardRobot(cleanbot, pix_delta, map, RIGHT_WALL_FOLLOWING_ENABLED);
                            travelled_dist += int(actual_pix);

                            readout2angle.clear();
                            angles.clear();
                        }
                    }
                }
                else if(cleanbot.right_readouts.back() > 0.0)
                {
                    RotateRobot(cleanbot, 2*rad_delta, map);
                }
            }
        }
    }
}

int main() {

    cv::Mat3b map(cv::Size(801, 801));
    cv::Mat canvas;

    map.setTo(cv::Vec3b(255, 255, 255));
    cv::circle(map, cv::Point(400, 400), 200, cv::Scalar(0, 0, 0), -1);
//    cv::circle(map, cv::Point(300, 400), 170, cv::Scalar(255, 255, 255), -1);

//    map.setTo(cv::Vec3b(0, 0, 0));
//    std::vector<cv::Point> contour1 = {cv::Point(10, 10), cv::Point(10, 790), cv::Point(790, 790), cv::Point(790, 10)};
//    std::vector<std::vector<cv::Point>> contours1 = {contour1};
//    cv::fillPoly(map, contours1, cv::Scalar(255, 255, 255));
//
//    std::vector<cv::Point> contour2 = {cv::Point(300, 300), cv::Point(300, 500), cv::Point(500, 500), cv::Point(500, 300)};
//    std::vector<std::vector<cv::Point>> contours2 = {contour2};
//    cv::fillPoly(map, contours2, cv::Scalar(0, 0, 0));

    cv::namedWindow("map", cv::WINDOW_NORMAL);

    Robot cleanbot = Robot(10, Point2D(20,20), Direction2D({1,1}));
    cv::LineIterator line(map, cv::Point(20, 20), cv::Point(800, 800));

    for(int i = 0; i < line.count-1; i++)
    {
        if(DetectCollision(map, cleanbot))
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