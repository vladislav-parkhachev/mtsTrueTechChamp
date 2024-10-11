#include <cpr/cpr.h>
#include <iostream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>

using json = nlohmann::json;

namespace api_interface
{
    const std::string token = "4326f9a3-1628-4884-90cb-c1b93890d61a8ffcd3c7-3620-4a0d-90b3-3dbe0b1c6267";
    const std::string matrix_send = "http://127.0.0.1:8801/api/v1/matrix/send?token=" + token;
    const std::string forward = "http://127.0.0.1:8801/api/v1/robot-cells/forward?token=" + token;
    const std::string backward = "http://127.0.0.1:8801/api/v1/robot-cells/backward?token=" + token;
    const std::string right = "http://127.0.0.1:8801/api/v1/robot-cells/right?token=" + token;
    const std::string left = "http://127.0.0.1:8801/api/v1/robot-cells/left?token=" + token;
    const std::string sensor_data = "http://127.0.0.1:8801/api/v1/robot-cells/sensor-data?token=" + token;
}

struct SensorsData
{
    float front_distance{};
    float right_side_distance{};
    float left_side_distance{};
    float back_distance{};
    float rotation_yaw{};
};

namespace move_robot
{
    void Forward();
    void Backward();
    void Left();
    void Right();
}

void getSensorsData(SensorsData &sensors_data);

int cellType(std::vector<double> &vals);

std::vector<std::vector<bool>> visited(16, std::vector<bool>(16));

template <typename T, std::size_t Row, std::size_t Col>
using Matrix = std::array<std::array<T, Col>, Row>;

template <typename T, std::size_t Row, std::size_t Col>
void sendMatrixMaze(const Matrix<T, Row, Col> &arr_matrix)
{
    json json_array(arr_matrix);
    auto string_matrix_maze = to_string(json_array);

    cpr::Response r = cpr::Post(cpr::Url{api_interface::matrix_send},
                                cpr::Body{string_matrix_maze},
                                cpr::Header{{"Content-Type", "application/json"}});
    std::cout << r.text << std::endl;
}

int main()
{
    Matrix<int, 16, 16> arr_matrix{{
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
        {10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10},
    }};

    // sendMatrixMaze(arr_matrix);

    SensorsData current_sensors_data;
    getSensorsData(current_sensors_data);
}

void getSensorsData(SensorsData &sensors_data)
{
    auto r = cpr::Get(cpr::Url{api_interface::sensor_data});
    json json_array = json::parse(r.text);
    sensors_data.front_distance = json_array["front_distance"];
    sensors_data.right_side_distance = json_array["right_side_distance"];
    sensors_data.left_side_distance = json_array["left_side_distance"];
    sensors_data.back_distance = json_array["back_distance"];
    sensors_data.rotation_yaw = json_array["rotation_yaw"];
}

void move_robot::Forward()
{
    cpr::Response r = cpr::Post(cpr::Url{api_interface::forward},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void move_robot::Backward()
{
    cpr::Response r = cpr::Post(cpr::Url{api_interface::backward},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void move_robot::Right()
{
    cpr::Response r = cpr::Post(cpr::Url{api_interface::right},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
}

void move_robot::Left()
{
    cpr::Response r = cpr::Post(cpr::Url{api_interface::left},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
}

int cellType(std::vector<double> &vals)
{
    double front = vals[0], right = vals[1], left = vals[2], back = vals[3];
    double yaw = vals[7];
    double mem;
    if (yaw > 85 && yaw < 95)
    {
        mem = front;
        front = left;
        left = back;
        back = right;
        right = mem;
    }
    else if (yaw > -95 && yaw < -85)
    {
        mem = front;
        front = right;
        right = back;
        back = left;
        left = mem;
    }
    else if (yaw > 175 && yaw < 185)
    {
        std::swap(right, left);
        std::swap(back, front);
    }

    int tres = 70;
    if (front < tres || right < tres || left < tres || back < tres)
    {
        if (front < tres)
        {
            if (right < tres)
            {
                if (left < tres)
                {
                    if (back < tres)
                        return 15;
                    else
                        return 12;
                }
                else if (back < tres)
                    return 11;
                else
                    return 7;
            }
            else if (left < tres)
            {
                if (back < tres)
                    return 13;
                else
                    return 8;
            }
            else if (back < tres)
                return 10;
            else
                return 2;
        }
        else if (left < tres)
        {

            if (right < tres)
            {
                if (back < tres)
                    return 14;
                else
                    return 9;
            }
            else if (back < tres)
                return 5;
            else
                return 1;
        }
        else if (right < tres)
        {
            if (back < tres)
                return 6;
            else
                return 3;
        }
        else if (back < tres)
            return 4;
    }
    else
        return 0;

    return -1;
}