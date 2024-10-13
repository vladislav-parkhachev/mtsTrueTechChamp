#include <cpr/cpr.h>
#include <iostream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <stack>

using json = nlohmann::json;

template <typename T, std::size_t Row, std::size_t Col>
using Matrix = std::array<std::array<T, Col>, Row>;

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

// SensorsData rotationSensors(SensorsData sensors_data);
// int cellType(const SensorsData &rotation_sensor_data);

template <typename T, std::size_t Row, std::size_t Col>
void sendMatrixMaze(const Matrix<T, Row, Col> &arr_matrix);

namespace maze_solver
{
    class RobotAPI
    {
    private:
        std::string token;
        const std::string api_matrix_send = "http://127.0.0.1:8801/api/v1/matrix/send?token=" + token;
        const std::string api_forward = "http://127.0.0.1:8801/api/v1/robot-cells/forward?token=" + token;
        const std::string api_backward = "http://127.0.0.1:8801/api/v1/robot-cells/backward?token=" + token;
        const std::string api_righ = "http://127.0.0.1:8801/api/v1/robot-cells/right?token=" + token;
        const std::string api_left = "http://127.0.0.1:8801/api/v1/robot-cells/left?token=" + token;
        const std::string api_sensor_data = "http://127.0.0.1:8801/api/v1/robot-cells/sensor-data?token=" + token;

        int x;
        int y;
        char direction;

    public:
        struct SensorsData
        {
            float front_distance{};
            float right_side_distance{};
            float left_side_distance{};
            float back_distance{};
            float rotation_yaw{};
        };

        SensorsData sensors_data;
        SensorsData rotation_sensors;

        RobotAPI(std::string token) : token{token} {}
        ~RobotAPI() = default;

        void rotationSensors();

        void UpdateSensorsData();
        char GetDirection();

        void MoveForward(int x, int y, char direction);
        void TurnLeft();
        void TurnRight();

        void set_x(int x);
        void set_y(int y);
        void set_direction(char direction);

        int get_x() const;
        int get_y() const;
        char get_direction() const;
    };

    class Algorithm
    {
    public:
        bool path_found_;
        bool temp_goal_{false}, path_blocked{true};
        char current_direction_;
        std::array<int, 2> current_node_;
        std::array<int, 2> parent_node_;
        std::stack<std::array<int, 2>> stack_;
        std::array<std::array<int, 16>, 16> arr_matrix;
        maze_solver::RobotAPI robot;
        std::array<std::array<bool, 16>, 16> visited_node_;
        std::array<int, 2> goal1_, goal2_, goal3_, goal4_, end_goal_;

        Algorithm() : path_found_{false}, current_direction_{'N'},
                      current_node_{}, parent_node_{}, stack_{},
                      visited_node_{}, robot{api_interface::token}, arr_matrix{}
        {
        }

        ~Algorithm() = default;

        bool IsVisited(std::array<int, 2> cur_node);
        bool AddNeighbour(std::array<int, 2> cur_node, std::array<int, 2> neighbour);
        bool FindNeighbours(std::array<int, 2> cur_node);
        bool DFSAlgorithm(std::array<int, 2> start);
        void Navigate(std::array<int, 2> cur_node, std::array<int, 2> next_node);
        int cellType();
        void sendMatrixMaze();
    };
}

int main()
{
    maze_solver::Algorithm algo_test_navigate;
    std::array<int, 2> start{15, 0};
    algo_test_navigate.DFSAlgorithm(start);
    algo_test_navigate.sendMatrixMaze();
}

void maze_solver::Algorithm::sendMatrixMaze()
{
    json json_array(this->arr_matrix);
    auto string_matrix_maze = to_string(json_array);
    std::cout << string_matrix_maze << std::endl;
    cpr::Response r = cpr::Post(cpr::Url{api_interface::matrix_send},
                                cpr::Body{string_matrix_maze},
                                cpr::Header{{"Content-Type", "application/json"}});
    std::cout << r.text << std::endl;
}

void maze_solver::RobotAPI::rotationSensors()
{
    this->rotation_sensors.rotation_yaw = sensors_data.rotation_yaw;
    switch ((int)sensors_data.rotation_yaw)
    {
    case 90:
        this->rotation_sensors.front_distance = sensors_data.left_side_distance;
        this->rotation_sensors.left_side_distance = sensors_data.back_distance;
        this->rotation_sensors.back_distance = sensors_data.right_side_distance;
        this->rotation_sensors.right_side_distance = sensors_data.front_distance;
        break;
    case -180:
        this->rotation_sensors.front_distance = sensors_data.back_distance;
        this->rotation_sensors.left_side_distance = sensors_data.right_side_distance;
        this->rotation_sensors.back_distance = sensors_data.front_distance;
        this->rotation_sensors.right_side_distance = sensors_data.left_side_distance;
        break;
    case 180:
        this->rotation_sensors.front_distance = sensors_data.back_distance;
        this->rotation_sensors.left_side_distance = sensors_data.right_side_distance;
        this->rotation_sensors.back_distance = sensors_data.front_distance;
        this->rotation_sensors.right_side_distance = sensors_data.left_side_distance;
        break;
    case -90:
        this->rotation_sensors.front_distance = sensors_data.right_side_distance;
        this->rotation_sensors.left_side_distance = sensors_data.front_distance;
        this->rotation_sensors.back_distance = sensors_data.left_side_distance;
        this->rotation_sensors.right_side_distance = sensors_data.back_distance;
        break;
    case 0:
        this->rotation_sensors.front_distance = sensors_data.front_distance;
        this->rotation_sensors.left_side_distance = sensors_data.left_side_distance;
        this->rotation_sensors.back_distance = sensors_data.back_distance;
        this->rotation_sensors.right_side_distance = sensors_data.right_side_distance;
        break;
    default:
        break;
    }
}

int maze_solver::Algorithm::cellType()
{
    int distance_wall = 70;
    if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance > distance_wall && this->robot.rotation_sensors.left_side_distance > distance_wall && this->robot.rotation_sensors.back_distance > distance_wall)
    {
        return 0;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance > distance_wall && this->robot.rotation_sensors.left_side_distance<distance_wall &&this->robot.rotation_sensors.back_distance> distance_wall)
    {
        return 1;
    }
    else if (this->robot.rotation_sensors.front_distance<distance_wall &&this->robot.rotation_sensors.right_side_distance> distance_wall && this->robot.rotation_sensors.left_side_distance > distance_wall && this->robot.rotation_sensors.back_distance > distance_wall)
    {
        return 2;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance<distance_wall &&this->robot.rotation_sensors.left_side_distance> distance_wall && this->robot.rotation_sensors.back_distance > distance_wall)
    {
        return 3;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance > distance_wall && this->robot.rotation_sensors.left_side_distance > distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 4;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance > distance_wall && this->robot.rotation_sensors.left_side_distance < distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 5;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance<distance_wall &&this->robot.rotation_sensors.left_side_distance> distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 6;
    }
    else if (this->robot.rotation_sensors.front_distance<distance_wall &&this->robot.rotation_sensors.right_side_distance<distance_wall &&this->robot.rotation_sensors.left_side_distance> distance_wall &&this->robot.rotation_sensors.back_distance> distance_wall)
    {
        return 7;
    }
    else if (this->robot.rotation_sensors.front_distance<distance_wall &&this->robot.rotation_sensors.right_side_distance> distance_wall && this->robot.rotation_sensors.left_side_distance<distance_wall &&this->robot.rotation_sensors.back_distance> distance_wall)
    {
        return 8;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance < distance_wall && this->robot.rotation_sensors.left_side_distance<distance_wall &&this->robot.rotation_sensors.back_distance> distance_wall)
    {
        return 9;
    }
    else if (this->robot.rotation_sensors.front_distance<distance_wall &&this->robot.rotation_sensors.right_side_distance> distance_wall && this->robot.rotation_sensors.left_side_distance > distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 10;
    }
    else if (this->robot.rotation_sensors.front_distance < distance_wall && this->robot.rotation_sensors.right_side_distance<distance_wall &&this->robot.rotation_sensors.left_side_distance> distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 11;
    }
    else if (this->robot.rotation_sensors.front_distance < distance_wall && this->robot.rotation_sensors.right_side_distance < distance_wall && this->robot.rotation_sensors.left_side_distance<distance_wall &&this->robot.rotation_sensors.back_distance> distance_wall)
    {
        return 12;
    }
    else if (this->robot.rotation_sensors.front_distance<distance_wall &&this->robot.rotation_sensors.right_side_distance> distance_wall && this->robot.rotation_sensors.left_side_distance < distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 13;
    }
    else if (this->robot.rotation_sensors.front_distance > distance_wall && this->robot.rotation_sensors.right_side_distance < distance_wall && this->robot.rotation_sensors.left_side_distance < distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 14;
    }
    else if (this->robot.rotation_sensors.front_distance < distance_wall && this->robot.rotation_sensors.right_side_distance < distance_wall && this->robot.rotation_sensors.left_side_distance < distance_wall && this->robot.rotation_sensors.back_distance < distance_wall)
    {
        return 15;
    }

    return -1;
}

char maze_solver::RobotAPI::GetDirection()
{
    UpdateSensorsData();

    if (sensors_data.rotation_yaw == 90)
    {
        return 'E';
    }
    else if (sensors_data.rotation_yaw == -180 || sensors_data.rotation_yaw == 180)
    {
        return 'S';
    }
    else if (sensors_data.rotation_yaw == -90)
    {
        return 'W';
    }

    return 'N';
}

void maze_solver::RobotAPI::UpdateSensorsData()
{
    cpr::Response r = cpr::Get(cpr::Url{api_interface::sensor_data});
    json json_array = json::parse(r.text);
    this->sensors_data.front_distance = json_array["front_distance"];
    this->sensors_data.right_side_distance = json_array["right_side_distance"];
    this->sensors_data.left_side_distance = json_array["left_side_distance"];
    this->sensors_data.back_distance = json_array["back_distance"];
    this->sensors_data.rotation_yaw = json_array["rotation_yaw"];
}

void maze_solver::RobotAPI::MoveForward(int x, int y, char direction)
{
    cpr::Response r = cpr::Post(cpr::Url{this->api_forward},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
    this->set_x(x);
    this->set_y(y);
    this->set_direction(direction);

    std::this_thread::sleep_for(std::chrono::milliseconds(100));
}

void maze_solver::RobotAPI::TurnLeft()
{
    cpr::Response r = cpr::Post(cpr::Url{this->api_left},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
}

void maze_solver::RobotAPI::TurnRight()
{
    cpr::Response r = cpr::Post(cpr::Url{this->api_righ},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
}

void maze_solver::RobotAPI::set_x(int x)
{
    this->x = x;
}
void maze_solver::RobotAPI::set_y(int y)
{
    this->y = y;
}

void maze_solver::RobotAPI::set_direction(char direction)
{
    this->direction = direction;
}

int maze_solver::RobotAPI::get_x() const
{
    return this->x;
}

int maze_solver::RobotAPI::get_y() const
{
    return this->y;
}

char maze_solver::RobotAPI::get_direction() const
{
    return this->direction;
}

bool maze_solver::Algorithm::IsVisited(std::array<int, 2> cur_node)
{
    return this->visited_node_.at(cur_node.at(0)).at(cur_node.at(1));
}

bool maze_solver::Algorithm::AddNeighbour(std::array<int, 2> cur_node, std::array<int, 2> neighbour)
{
    if (!IsVisited(neighbour))
    {
        this->stack_.push(neighbour);
        return true;
    }
    return false;
}

bool maze_solver::Algorithm::FindNeighbours(std::array<int, 2> cur_node)
{
    char robotDirection{robot.GetDirection()};

    std::array<int, 2> node_N{cur_node[0] - 1, cur_node[1]},
        node_W{cur_node[0], cur_node[1] - 1},
        node_S{cur_node[0] + 1, cur_node[1]},
        node_E{cur_node[0], cur_node[1] + 1};

    robot.UpdateSensorsData();

    if (robotDirection == 'N')
    {
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.left_side_distance > 70)
            return AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.front_distance > 70)
            return AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.right_side_distance > 70)
            return AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.back_distance > 70)
            return AddNeighbour(cur_node, node_S);
    }
    else if (robotDirection == 'S')
    {
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.left_side_distance > 70)
            return AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.front_distance > 70)
            return AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.right_side_distance > 70)
            return AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.back_distance > 70)
            return AddNeighbour(cur_node, node_N);
    }
    else if (robotDirection == 'E')
    {
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.left_side_distance > 70)
            return AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.front_distance > 70)
            return AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.right_side_distance > 70)
            return AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.back_distance > 70)
            return AddNeighbour(cur_node, node_W);
    }
    else if (!this->temp_goal_ && robotDirection == 'W')
    {
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.left_side_distance > 70)
            return AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.front_distance > 70)
            return AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.right_side_distance > 70)
            return AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.back_distance > 70)
            return AddNeighbour(cur_node, node_E);
    }

    return false;
}

bool maze_solver::Algorithm::DFSAlgorithm(std::array<int, 2> start)
{
    std::array<int, 2> curr_node{};
    curr_node = start;
    this->temp_goal_ = false;
    this->stack_.push(start);
    int count = 1;
    this->robot.UpdateSensorsData();
    this->robot.rotationSensors();
    this->arr_matrix.at(curr_node.at(0)).at(curr_node.at(1)) = this->cellType();
    
    while (count < 257)
    {
        if (FindNeighbours(curr_node))
        {
            count++;
            this->visited_node_[curr_node[0]][curr_node[1]] = true;
            this->Navigate(curr_node, this->stack_.top());
        }
        else
        {
            this->stack_.pop();
            this->Navigate(curr_node, this->stack_.top());
        }

        curr_node = stack_.top();
        this->robot.UpdateSensorsData();
        this->robot.rotationSensors();
        this->arr_matrix.at(curr_node.at(0)).at(curr_node.at(1)) = this->cellType();

    }
    return false;
}

void maze_solver::Algorithm::Navigate(std::array<int, 2> cur_node, std::array<int, 2> next_node)
{
    int x{}, y{};
    char curr_direction{};
    char direction_togo{};
    std::array<int, 2> node_curr{}, node_next{};

    node_next = next_node;
    node_curr = cur_node;

    x = node_next[0] - node_curr[0];
    y = node_next[1] - node_curr[1];
    curr_direction = this->robot.GetDirection();

    if (x == -1 && y == 0)
        direction_togo = 'N';
    else if (x == 1 && y == 0)
        direction_togo = 'S';
    else if (x == 0 && y == -1)
        direction_togo = 'W';
    else if (x == 0 && y == 1)
        direction_togo = 'E';

    this->robot.UpdateSensorsData();

    if (curr_direction == 'N')
    {
        if (direction_togo == 'N')
        {
            if (this->robot.sensors_data.front_distance > 70)
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'S')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
        }
        else if (direction_togo == 'E')
        {
            if (this->robot.sensors_data.right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'W')
        {
            if (this->robot.sensors_data.left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
    }
    else if (curr_direction == 'S')
    {
        if (direction_togo == 'N')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
        }
        else if (direction_togo == 'S')
        {
            if (this->robot.sensors_data.front_distance > 70)
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'E')
        {
            if (this->robot.sensors_data.left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'W')
        {
            if (this->robot.sensors_data.right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
    }
    else if (curr_direction == 'E')
    {
        if (direction_togo == 'N')
        {
            if (this->robot.sensors_data.left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'S')
        {
            if (this->robot.sensors_data.right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'E')
        {
            if (this->robot.sensors_data.front_distance > 70)
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'W')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
        }
    }
    else if (curr_direction == 'W')
    {
        if (direction_togo == 'N')
        {
            if (this->robot.sensors_data.right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'S')
        {
            if (this->robot.sensors_data.left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            }
            else
                this->path_blocked = true;
        }
        else if (direction_togo == 'E')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
        }
        else if (direction_togo == 'W')
        {
            if (this->robot.sensors_data.front_distance > 70)
                this->robot.MoveForward(node_next[0], node_next[1], direction_togo);
            else
                this->path_blocked = true;
        }
    }
}