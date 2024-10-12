#include <cpr/cpr.h>
#include <iostream>
#include <algorithm>
#include <nlohmann/json.hpp>
#include <thread>
#include <chrono>
#include <stack>
#include <vector>
#include <tuple>

using json = nlohmann::json;

template <typename T, std::size_t Row, std::size_t Col>
using Matrix = std::array<std::array<T, Col>, Row>;

int gotPoints = 0;
int distance_wall = 70;
std::stack<std::tuple<char, int, int>> moveHistory;
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
// int cellType(SensorsData sensors_data);
SensorsData rotationSensors(SensorsData sensors_data);
void printSensorsData(const SensorsData &sensors_data);
int cellType(const SensorsData &rotation_sensor_data);

template <typename T, std::size_t Row, std::size_t Col>
void moveRobot(const SensorsData &sensots_data, const Matrix<T, Row, Col> &visited_matrix, int &row, int &col);

// int cellType(std::vector<double> &vals);

// std::array<std::array<bool, 16>, 16> visited;

template <typename T, std::size_t Row, std::size_t Col>
void sendMatrixMaze(const Matrix<T, Row, Col> &arr_matrix);

template <typename T, std::size_t Row, std::size_t Col>
int explore(int &x, int &y, SensorsData &current_sensors_data, const Matrix<T, Row, Col> &visited);

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

        RobotAPI(std::string token) : token{token} {}
        ~RobotAPI() = default;

        void UpdateSensorsData();
        char GetDirection();
    };

    struct Node
    {
        int dist_;
        std::array<int, 2> parent_node_;
        Node() : dist_{}, parent_node_{} {}
        ~Node() = default;
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
        maze_solver::RobotAPI robot;
        std::stack<std::array<int, 2>> path_stack_;
        Matrix<Node, 16, 16> node_info;
        Matrix<Node, 16, 16> node_master_;
        Matrix<bool, 16, 16> explored_node_;
        Matrix<bool, 16, 16> visited_node_;
        std::array<int, 2> goal1_, goal2_, goal3_, goal4_, end_goal_;

        Algorithm() : path_found_{false}, current_direction_{'N'},
                      current_node_{}, parent_node_{}, stack_{}, path_stack_{},
                      explored_node_{}, visited_node_{}, node_master_{}, robot{api_interface::token}, node_info{}
        {
        }

        ~Algorithm() = default;

        bool IsExplored(std::array<int, 2> cur_node);
        bool IsVisited(std::array<int, 2> cur_node);
        bool AddNeighbour(std::array<int, 2> cur_node, std::array<int, 2> neighbour);
        void FindNeighbours(std::array<int, 2> cur_node);
        bool DFSAlgorithm(std::array<int, 2> start);
        std::stack<std::array<int, 2>> BackTrack(std::array<int, 2> current_node, Matrix<Node, 16, 16> &node);
        void Solve();
        void Navigate(std::stack<std::array<int, 2>> &path);
        void ClearStack();
        void SetDefaults();
    };

}

int main()
{
    maze_solver::Algorithm test_algorithm;
    std::array<int, 2> cur_node{15, 2};
    test_algorithm.FindNeighbours(cur_node);
    while (!test_algorithm.stack_.empty())
    {
        std::cout << test_algorithm.stack_.top().at(0) << std::endl;
        std::cout << test_algorithm.stack_.top().at(1) << std::endl;
        test_algorithm.stack_.pop();
    }

    // Matrix<int, 16, 16> arr_matrix{};
    // Matrix<bool, 16, 16> visited_matrix{};

    // SensorsData current_sensors_data;
    // SensorsData rotation_sensors_data;

    // int row = 15, col = 0;
    // int count = 1;

    // getSensorsData(current_sensors_data);
    //
    // getSensorsData(current_sensors_data);

    // while (count < 257)
    // {
    //     rotation_sensors_data = rotationSensors(current_sensors_data);
    //     arr_matrix.at(row).at(col) = cellType(rotation_sensors_data);

    //     if (arr_matrix.at(row).at(col) == 0)
    //     {
    //         count++;
    //     }

    //     visited_matrix.at(row).at(col) = 1;
    //     std::cout << "row: " << row << "  col: " << col << std::endl;
    //     printSensorsData(current_sensors_data);
    //     moveRobot(current_sensors_data, visited_matrix, row, col);
    //     getSensorsData(current_sensors_data);
    // }

    // sendMatrixMaze(arr_matrix);

    // json json_array(arr_matrix);
    // auto string_matrix_maze = to_string(json_array);
    // std::cout << string_matrix_maze << std::endl;

    // if(visited_matrix.at(row).at(col) == 0)
    // {
    //     moveRobot(current_sensors_data, visited_matrix, row, col);
    // }

    // visited_matrix.at(row).at(col) = 1;
    // }

    //     for (auto& row : arr_matrix) {
    //         std::fill(row.begin(), row.end(), -1);
    //     }

    //     while(gotPoints<256){
    //         getSensorsData(current_sensors_data);
    //         rotation_sensors_data = rotationSensors(current_sensors_data);
    //         arr_matrix[x][y] = cellType(rotation_sensors_data);
    //         visited[x][y]=true;
    //         if(explore(x,y,current_sensors_data,visited)==2) break;
    //         // std::cout<<x<<" "<<y<<"   ";
    //     }
    //     for (size_t i = 0; i < arr_matrix.size(); ++i) {
    //         for (size_t j = 0; j < arr_matrix[i].size(); ++j) {
    //             transposed_arr_matrix[j][i] = arr_matrix[i][j];
    //         }
    //     }
    //     sendMatrixMaze(transposed_arr_matrix);
    //     std::cout<<gotPoints<<std::endl;
    //     while (!moveHistory.empty()) {
    //         auto item = moveHistory.top(); // Получаем верхний элемент
    //         std::cout << "Элемент: ("
    //                   << std::get<0>(item) << ", "
    //                   << std::get<1>(item) << ", "
    //                   << std::get<2>(item) << ")" <<"  ";
    //         moveHistory.pop(); // Удаляем верхний элемент
    //     }
    //     std::cout<<std::endl;
}
template <typename T, std::size_t Row, std::size_t Col>
bool isRightCellNotVis(int x, int y, const Matrix<T, Row, Col> &visited, const std::vector<std::pair<int, int>> &directions)
{
    return !visited[x + directions[1].first][y + directions[1].second];
}
template <typename T, std::size_t Row, std::size_t Col>
bool isFrontCellNotVis(int x, int y, const Matrix<T, Row, Col> &visited, const std::vector<std::pair<int, int>> &directions)
{
    return !visited[x + directions[0].first][y + directions[0].second];
}
template <typename T, std::size_t Row, std::size_t Col>
bool isLeftCellNotVis(int x, int y, const Matrix<T, Row, Col> &visited, const std::vector<std::pair<int, int>> &directions)
{
    return !visited[x + directions[3].first][y + directions[3].second];
}
template <typename T, std::size_t Row, std::size_t Col>
int explore(int &x, int &y, SensorsData &current_sensors_data, const Matrix<T, Row, Col> &visited)
{
    std::vector<std::pair<int, int>> directions;
    std::cout << current_sensors_data.rotation_yaw << " \n";
    switch ((int)current_sensors_data.rotation_yaw)
    { // вперёд,вправо,назад,влево
    case 90:
        directions = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        break;
    case -180:
        directions = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
        break;
    case 180:
        directions = {{0, 1}, {-1, 0}, {0, -1}, {1, 0}};
        break;
    case -90:
        directions = {{-1, 0}, {0, -1}, {1, 0}, {0, 1}};
        break;
    case -1 ... 1:
        directions = {{0, -1}, {1, 0}, {0, 1}, {-1, 0}};
        break;
    default:
        break;
    }
    bool front = current_sensors_data.front_distance > distance_wall,
         right = current_sensors_data.right_side_distance > distance_wall,
         left = current_sensors_data.left_side_distance > distance_wall,
         back = current_sensors_data.back_distance > distance_wall;
    std::cout << front << " " << right << " " << left << " " << back << std::endl;
    if (right && isRightCellNotVis(x, y, visited, directions))
    { // right
        moveHistory.push(std::make_tuple('r', x, y));
        move_robot::Right();
        move_robot::Forward();
        gotPoints += 1;
        x += directions[1].first;
        y += directions[1].second;
        return 0;
    }
    else if (front && isFrontCellNotVis(x, y, visited, directions))
    { // front
        moveHistory.push(std::make_tuple('f', x, y));
        move_robot::Forward();
        gotPoints += 1;
        x += directions[0].first;
        y += directions[0].second;
        return 0;
    }
    else if (left && isLeftCellNotVis(x, y, visited, directions))
    { // left
        moveHistory.push(std::make_tuple('l', x, y));
        move_robot::Left();
        move_robot::Forward();
        gotPoints += 1;
        x += directions[3].first;
        y += directions[3].second;
        return 0;
    }
    else if (back)
    {
        // move_robot::Backward();НЕ работает
        // auto item = moveHistory.top();
        // char dir= std::get<0>(item);
        // int hisX = std::get<1>(item);
        // int hisY = std::get<2>(item);
        // x = hisX;
        // y = hisY;
        // while(!((right && isRightCellNotVis(x,y,visited,directions)) || (front && isFrontCellNotVis(x,y,visited,directions)) || (left && isLeftCellNotVis(x,y,visited,directions)))){
        //     moveHistory.pop();
        //     auto item = moveHistory.top();
        //     char dir = std::get<0>(item);
        //     int hisX = std::get<1>(item);
        //     int hisY = std::get<2>(item);
        //     switch(dir){
        //         case 'r':
        //             move_robot::Forward();
        //             move_robot::Left();
        //             break;
        //         case 'f':
        //             move_robot::Forward();
        //             break;
        //         case 'l':
        //             move_robot::Forward();
        //             move_robot::Right();
        //             break;
        //     }
        //     x=hisX;
        //     y=hisY;
        // }
        // std::cout<<"x: "<<x<<" y: "<<y<<std::endl;
        return 1;
    }
    else
        return 1;
}

template <typename T, std::size_t Row, std::size_t Col>
void moveRobot(const SensorsData &sensots_data, const Matrix<T, Row, Col> &visited_matrix, int &row, int &col)
{
    if (sensots_data.right_side_distance > 70)
    {
        switch ((int)sensots_data.rotation_yaw)
        {
        case 90:
            row++;
            break;
        case -180:
            col--;
            break;
        case 180:
            col--;
            break;
        case -90:
            row--;
            break;
        case 0:
            col++;
            break;
        }

        move_robot::Right();
    }

    else if (sensots_data.front_distance > 70)
    {
        switch ((int)sensots_data.rotation_yaw)
        {
        case 90:
            col++;
            break;
        case -180:
            row++;
            break;
        case 180:
            row++;
            break;
        case -90:
            col--;
            break;
        case 0:
            row--;
            break;
        }
    }

    else if (sensots_data.left_side_distance > 70)
    {
        switch ((int)sensots_data.rotation_yaw)
        {
        case 90:
            row--;
            break;
        case -180:
            col++;
            break;
        case 180:
            col++;
            break;
        case -90:
            row++;
            break;
        case 0:
            col--;
            break;
        }

        move_robot::Left();
    }
    else if (sensots_data.back_distance > 70)
    {
        move_robot::Right();
        move_robot::Right();
    }

    move_robot::Forward();
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

template <typename T, std::size_t Row, std::size_t Col>
void sendMatrixMaze(const Matrix<T, Row, Col> &arr_matrix)
{
    json json_array(arr_matrix);
    auto string_matrix_maze = to_string(json_array);
    std::cout << string_matrix_maze << std::endl;
    cpr::Response r = cpr::Post(cpr::Url{api_interface::matrix_send},
                                cpr::Body{string_matrix_maze},
                                cpr::Header{{"Content-Type", "application/json"}});
    std::cout << r.text << std::endl;
}

void move_robot::Forward()
{
    cpr::Response r = cpr::Post(cpr::Url{api_interface::forward},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
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

// void move_robot::Backward()
// {
//     move_robot::Right();
//     move_robot::Right();
//     move_robot::Forward();
// }

SensorsData rotationSensors(SensorsData sensors_data)
{
    SensorsData rotation_sensors;
    rotation_sensors.rotation_yaw = sensors_data.rotation_yaw;
    switch ((int)sensors_data.rotation_yaw)
    {
    case 90:
        rotation_sensors.front_distance = sensors_data.left_side_distance;
        rotation_sensors.left_side_distance = sensors_data.back_distance;
        rotation_sensors.back_distance = sensors_data.right_side_distance;
        rotation_sensors.right_side_distance = sensors_data.front_distance;
        break;
    case -180:
        rotation_sensors.front_distance = sensors_data.back_distance;
        rotation_sensors.left_side_distance = sensors_data.right_side_distance;
        rotation_sensors.back_distance = sensors_data.front_distance;
        rotation_sensors.right_side_distance = sensors_data.left_side_distance;
        break;
    case 180:
        rotation_sensors.front_distance = sensors_data.back_distance;
        rotation_sensors.left_side_distance = sensors_data.right_side_distance;
        rotation_sensors.back_distance = sensors_data.front_distance;
        rotation_sensors.right_side_distance = sensors_data.left_side_distance;
        break;
    case -90:
        rotation_sensors.front_distance = sensors_data.right_side_distance;
        rotation_sensors.left_side_distance = sensors_data.front_distance;
        rotation_sensors.back_distance = sensors_data.left_side_distance;
        rotation_sensors.right_side_distance = sensors_data.back_distance;
        break;
    case 0:
        rotation_sensors.front_distance = sensors_data.front_distance;
        rotation_sensors.left_side_distance = sensors_data.left_side_distance;
        rotation_sensors.back_distance = sensors_data.back_distance;
        rotation_sensors.right_side_distance = sensors_data.right_side_distance;
        break;
    default:
        break;
    }

    return rotation_sensors;
}

void printSensorsData(const SensorsData &sensors_data)
{
    std::cout << "front_distance: " << sensors_data.front_distance << std::endl;
    std::cout << "right_side_distance: " << sensors_data.right_side_distance << std::endl;
    std::cout << "left_side_distance: " << sensors_data.left_side_distance << std::endl;
    std::cout << "back_distance: " << sensors_data.back_distance << std::endl;
    std::cout << "rotation_yaw: " << sensors_data.rotation_yaw << std::endl;
}

int cellType(const SensorsData &rotation_sensor_data)
{

    if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 0;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 1;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 2;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 3;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 4;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 5;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 6;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 7;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 8;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 9;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 10;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance > distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 11;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance > distance_wall)
    {
        return 12;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance > distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 13;
    }
    else if (rotation_sensor_data.front_distance > distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance < distance_wall)
    {
        return 14;
    }
    else if (rotation_sensor_data.front_distance < distance_wall && rotation_sensor_data.right_side_distance < distance_wall && rotation_sensor_data.left_side_distance < distance_wall && rotation_sensor_data.back_distance < distance_wall)
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

void maze_solver::Algorithm::ClearStack()
{
    while (!this->stack_.empty())
        this->stack_.pop();
}

bool maze_solver::Algorithm::IsExplored(std::array<int, 2> cur_node)
{
    return this->explored_node_.at(cur_node.at(0)).at(cur_node.at(1));
}

bool maze_solver::Algorithm::IsVisited(std::array<int, 2> cur_node)
{
    return this->visited_node_.at(cur_node.at(0)).at(cur_node.at(1));
}

bool maze_solver::Algorithm::AddNeighbour(std::array<int, 2> cur_node, std::array<int, 2> neighbour)
{
    if (!IsExplored(neighbour))
    {
        this->stack_.push(neighbour);
        this->node_info[neighbour[0]][neighbour[1]].parent_node_ = cur_node;
        if (neighbour == this->goal1_ || neighbour == this->goal2_ ||
            neighbour == this->goal3_ || neighbour == this->goal4_)
        {
            this->temp_goal_ = true;
            return true;
        }
    }
    return false;
}

void maze_solver::Algorithm::FindNeighbours(std::array<int, 2> cur_node)
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
            AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.front_distance > 70)
            AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.right_side_distance > 70)
            AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.back_distance > 70)
            AddNeighbour(cur_node, node_S);
    }
    else if (robotDirection == 'S')
    {
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.left_side_distance > 70)
            AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.front_distance > 70)
            AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.right_side_distance > 70)
            AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.back_distance > 70)
            AddNeighbour(cur_node, node_N);
    }
    else if (robotDirection == 'E')
    {
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.left_side_distance > 70)
            AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.front_distance > 70)
            AddNeighbour(cur_node, node_E);
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.right_side_distance > 70)
            AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.back_distance > 70)
            AddNeighbour(cur_node, node_W);
    }
    else if (!this->temp_goal_ && robotDirection == 'W')
    {
        if (!this->temp_goal_ && node_S[0] <= 15 && robot.sensors_data.right_side_distance > 70)
            AddNeighbour(cur_node, node_S);
        if (!this->temp_goal_ && node_W[1] >= 0 && robot.sensors_data.front_distance > 70)
            AddNeighbour(cur_node, node_W);
        if (!this->temp_goal_ && node_N[0] >= 0 && robot.sensors_data.right_side_distance > 70)
            AddNeighbour(cur_node, node_N);
        if (!this->temp_goal_ && node_E[1] <= 15 && robot.sensors_data.back_distance > 70)
            AddNeighbour(cur_node, node_E);
    }
}


