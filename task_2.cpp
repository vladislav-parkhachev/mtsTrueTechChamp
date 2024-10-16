#include <cpr/cpr.h>
#include <iostream>
#include <stack>

namespace maze_solver
{
    class RobotAPI
    {
    private:

        struct SensorsData
        {
            float front_distance{};
            float right_side_distance{};
            float left_side_distance{};
            float back_distance{};
            float left_45_distance{};
            float right_45_distance{};
            float rotation_yaw{};
        };

        SensorsData current_sensors_data{};
        SensorsData rotation_sensors_data{};

        void parseSensorsData(const std::string &string_sensors_data);
        char current_robot_direction{'N'};

    public:

        RobotAPI() {}
        ~RobotAPI() {}

        std::string token{"4326f9a3-1628-4884-90cb-c1b93890d61a8ffcd3c7-3620-4a0d-90b3-3dbe0b1c6267"};

        void updateSensorsData();
        SensorsData getCurrentSensorsData() const;
        void rotationCurrentSensorsData();
        void updateRobotDirection();
        char getCurrentRobotDirection() const;
        SensorsData getRotationSensorsData() const;

        void MoveForward();
        void TurnLeft();
        void TurnRight();
    };

    class Algorithm
    {
    private:
        std::array<int, 2> next_cell{};
        
        std::array<std::array<bool, 16>, 16> visited_cell{};
        std::array<std::array<int, 16>, 16> maze_map{};
        std::string string_maze_map{};
        
        
        bool neighbor_added;

        void convertMatrixToString();

        std::array<int, 2> goal1_{7, 7}, goal2_{7, 8}, goal3_{7, 8}, goal4_{8, 8};

    public:
        Algorithm() {}
        ~Algorithm() {}
        maze_solver::RobotAPI robot{};
        std::stack<std::array<int, 2>> move_history{};
        std::stack<std::array<int, 2>> move_history_revers;
        std::array<int, 2> current_cell{15, 0};

        bool IsCellVisited(std::array<int, 2> cell);
        void FindNeighbours();
        void AddNeighbour(std::array<int, 2> neighbour);
        void DFSAlgorithm();
        void Navigate(std::array<int, 2> next_node);
        int cellType();
        void sendMazeMap();
        std::array<int, 2> DFSAlgorithmFinish();
    };
}

int main()
{
    maze_solver::Algorithm algo_test_navigate;

    algo_test_navigate.DFSAlgorithmFinish();

    std::string maze_restart = "http://127.0.0.1:8801/api/v1/maze/restart?token=" + algo_test_navigate.robot.token;
    cpr::Response r = cpr::Post(cpr::Url{maze_restart},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});

    while (!algo_test_navigate.move_history.empty())
    {
        algo_test_navigate.move_history_revers.push(algo_test_navigate.move_history.top());
        algo_test_navigate.move_history.pop();
    }

    // algo_test_navigate.current_cell.at(0) = 15;
    // algo_test_navigate.current_cell.at(1) = 0;

    algo_test_navigate.robot.updateSensorsData();
    algo_test_navigate.robot.updateRobotDirection();

    while (algo_test_navigate.move_history_revers.size() > 1)
    {
        algo_test_navigate.current_cell = algo_test_navigate.move_history_revers.top();
        algo_test_navigate.move_history_revers.pop();
        algo_test_navigate.Navigate(algo_test_navigate.move_history_revers.top());
        algo_test_navigate.robot.updateSensorsData();
        algo_test_navigate.robot.updateRobotDirection();
    }
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::updateSensorsData()
{
    std::string api_get_sensors_data{"http://127.0.0.1:8801/api/v1/robot-cells/sensor-data?token=" + token};
    cpr::Response response = cpr::Get(cpr::Url{api_get_sensors_data});
    parseSensorsData(response.text);
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::parseSensorsData(const std::string &string_sensors_data)
{
    std::array<std::string, 11> params = {
        "front_distance",
        "right_side_distance",
        "left_side_distance",
        "back_distance",
        "left_45_distance",
        "right_45_distance",
        "rotation_pitch",
        "rotation_yaw",
        "rotation_roll",
        "down_x_offset",
        "down_y_offset"};

    std::array<float, 11> vals{};
    for (int i = 0; i < params.size() - 1; ++i)
    {
        int beg = string_sensors_data.find(params[i]) + params[i].size() + 2;
        int end = string_sensors_data.find(params[i + 1]) - 2;
        std::string res = string_sensors_data.substr(beg, end - beg);
        vals[i] = std::stod(res);
    }
    int lastbeg = string_sensors_data.find(params[params.size() - 1]) + params[params.size() - 1].size() + 2;
    int lastend = string_sensors_data.find("}") - 1;
    std::string lastres = string_sensors_data.substr(lastbeg, lastend - lastbeg);
    vals[vals.size() - 1] = std::stod(lastres);

    this->current_sensors_data.front_distance = vals[0];
    this->current_sensors_data.right_side_distance = vals[1];
    this->current_sensors_data.left_side_distance = vals[2];
    this->current_sensors_data.back_distance = vals[3];
    this->current_sensors_data.left_45_distance = vals[4];
    this->current_sensors_data.right_45_distance = vals[5];
    this->current_sensors_data.rotation_yaw = vals[7];
}

/* ==================================================================================================== */

maze_solver::RobotAPI::SensorsData maze_solver::RobotAPI::getCurrentSensorsData() const
{
    return current_sensors_data;
}

/* ==================================================================================================== */

maze_solver::RobotAPI::SensorsData maze_solver::RobotAPI::getRotationSensorsData() const
{
    return rotation_sensors_data;
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::rotationCurrentSensorsData()
{
    this->rotation_sensors_data.rotation_yaw = this->current_sensors_data.rotation_yaw;
    switch ((int)rotation_sensors_data.rotation_yaw)
    {
    case 90:
        this->rotation_sensors_data.front_distance = current_sensors_data.left_side_distance;
        this->rotation_sensors_data.left_side_distance = current_sensors_data.back_distance;
        this->rotation_sensors_data.back_distance = current_sensors_data.right_side_distance;
        this->rotation_sensors_data.right_side_distance = current_sensors_data.front_distance;
        break;
    case -180:
        this->rotation_sensors_data.front_distance = current_sensors_data.back_distance;
        this->rotation_sensors_data.left_side_distance = current_sensors_data.right_side_distance;
        this->rotation_sensors_data.back_distance = current_sensors_data.front_distance;
        this->rotation_sensors_data.right_side_distance = current_sensors_data.left_side_distance;
        break;
    case 180:
        this->rotation_sensors_data.front_distance = current_sensors_data.back_distance;
        this->rotation_sensors_data.left_side_distance = current_sensors_data.right_side_distance;
        this->rotation_sensors_data.back_distance = current_sensors_data.front_distance;
        this->rotation_sensors_data.right_side_distance = current_sensors_data.left_side_distance;
        break;
    case -90:
        this->rotation_sensors_data.front_distance = current_sensors_data.right_side_distance;
        this->rotation_sensors_data.left_side_distance = current_sensors_data.front_distance;
        this->rotation_sensors_data.back_distance = current_sensors_data.left_side_distance;
        this->rotation_sensors_data.right_side_distance = current_sensors_data.back_distance;
        break;
    case 0:
        this->rotation_sensors_data.front_distance = current_sensors_data.front_distance;
        this->rotation_sensors_data.left_side_distance = current_sensors_data.left_side_distance;
        this->rotation_sensors_data.back_distance = current_sensors_data.back_distance;
        this->rotation_sensors_data.right_side_distance = current_sensors_data.right_side_distance;
        break;
    default:
        break;
    }
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::updateRobotDirection()
{
    if (this->current_sensors_data.rotation_yaw == 0)
    {
        this->current_robot_direction = 'N';
    }
    else if (this->current_sensors_data.rotation_yaw == 90)
    {
        this->current_robot_direction = 'E';
    }
    else if (this->current_sensors_data.rotation_yaw == -180 || this->current_sensors_data.rotation_yaw == 180)
    {
        this->current_robot_direction = 'S';
    }
    else if (this->current_sensors_data.rotation_yaw == -90)
    {
        this->current_robot_direction = 'W';
    }
}

/* ==================================================================================================== */

char maze_solver::RobotAPI::getCurrentRobotDirection() const
{
    return this->current_robot_direction;
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::MoveForward()
{
    const std::string api_forward{"http://127.0.0.1:8801/api/v1/robot-cells/forward?token=" + this->token};
    cpr::Response r = cpr::Post(cpr::Url{api_forward},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::TurnLeft()
{
    const std::string api_turn_left{"http://127.0.0.1:8801/api/v1/robot-cells/left?token=" + this->token};
    cpr::Response r = cpr::Post(cpr::Url{api_turn_left},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});

    std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

/* ==================================================================================================== */

void maze_solver::RobotAPI::TurnRight()
{
    const std::string api_turn_righ{"http://127.0.0.1:8801/api/v1/robot-cells/right?token=" + this->token};
    cpr::Response r = cpr::Post(cpr::Url{api_turn_righ},
                                cpr::Body{""},
                                cpr::Header{{"accept", "application/json"}});
                                
    std::this_thread::sleep_for(std::chrono::milliseconds(150));
}

/* ==================================================================================================== */

bool maze_solver::Algorithm::IsCellVisited(std::array<int, 2> cell)
{
    return this->visited_cell.at(cell.at(0)).at(cell.at(1));
}

/* ==================================================================================================== */

std::array<int, 2> maze_solver::Algorithm::DFSAlgorithmFinish()
{
    std::array<int, 2> finish_node;
    this->move_history.push(this->current_cell);
    this->robot.updateSensorsData();
    this->robot.rotationCurrentSensorsData();

    while (true)
    {
        std::cout << this->current_cell.at(0)<< "  "<< this->current_cell.at(1) << '\n';
        if (this->current_cell == this->goal1_ || this->current_cell == this->goal2_ ||
            this->current_cell == this->goal3_ || this->current_cell == this->goal4_)
        {
            finish_node = this->current_cell;
            break;
        }

        FindNeighbours();
        if (this->neighbor_added)
        {
            if (this->move_history.top() == goal1_ || move_history.top() == goal2_ || move_history.top() == goal3_ || move_history.top() == goal4_)
            {
                break;
            }
            this->visited_cell[this->current_cell[0]][this->current_cell[1]] = true;
            this->Navigate(this->next_cell);
        }
        else
        {
            this->move_history.pop();
            this->Navigate(this->move_history.top());
        }

        this->current_cell = move_history.top();
        this->robot.updateSensorsData();
        this->robot.updateRobotDirection();
    }
    return finish_node;
}

/* ==================================================================================================== */

void maze_solver::Algorithm::FindNeighbours()
{
    std::array<int, 2> cell_N{this->current_cell[0] - 1, this->current_cell[1]},
        cell_W{this->current_cell[0], this->current_cell[1] - 1},
        cell_S{this->current_cell[0] + 1, this->current_cell[1]},
        cell_E{this->current_cell[0], this->current_cell[1] + 1};

    if (this->robot.getCurrentRobotDirection() == 'N')
    {
        if (cell_W[1] >= 0 && this->robot.getCurrentSensorsData().left_side_distance > 70)
            AddNeighbour(cell_W);
        else if (cell_N[0] >= 0 && this->robot.getCurrentSensorsData().front_distance > 70)
            AddNeighbour(cell_N);
        else if (cell_E[1] <= 15 && this->robot.getCurrentSensorsData().right_side_distance > 70)
            AddNeighbour(cell_E);
        else if (cell_S[0] <= 15 && this->robot.getCurrentSensorsData().back_distance > 70)
            AddNeighbour(cell_S);
    }
    else if (this->robot.getCurrentRobotDirection() == 'S')
    {
        if (cell_E[1] <= 15 && this->robot.getCurrentSensorsData().left_side_distance > 70)
            AddNeighbour(cell_E);
        else if (cell_S[0] <= 15 && this->robot.getCurrentSensorsData().front_distance > 70)
            AddNeighbour(cell_S);
        else if (cell_W[1] >= 0 && this->robot.getCurrentSensorsData().right_side_distance > 70)
            AddNeighbour(cell_W);
        else if (cell_N[0] >= 0 && this->robot.getCurrentSensorsData().back_distance > 70)
            AddNeighbour(cell_N);
    }
    else if (this->robot.getCurrentRobotDirection() == 'E')
    {
        if (cell_N[0] >= 0 && this->robot.getCurrentSensorsData().left_side_distance > 70)
            AddNeighbour(cell_N);
        else if (cell_E[1] <= 15 && this->robot.getCurrentSensorsData().front_distance > 70)
            AddNeighbour(cell_E);
        else if (cell_S[0] <= 15 && this->robot.getCurrentSensorsData().right_side_distance > 70)
            AddNeighbour(cell_S);
        else if (cell_W[1] >= 0 && this->robot.getCurrentSensorsData().back_distance > 70)
            AddNeighbour(cell_W);
    }
    else if (this->robot.getCurrentRobotDirection() == 'W')
    {
        if (cell_S[0] <= 15 && this->robot.getCurrentSensorsData().left_side_distance > 70)
            AddNeighbour(cell_S);
        else if (cell_W[1] >= 0 && this->robot.getCurrentSensorsData().front_distance > 70)
            AddNeighbour(cell_W);
        else if (cell_N[0] >= 0 && this->robot.getCurrentSensorsData().right_side_distance > 70)
            AddNeighbour(cell_N);
        else if (cell_E[1] <= 15 && this->robot.getCurrentSensorsData().back_distance > 70)
            AddNeighbour(cell_E);
    }
}

/* ==================================================================================================== */

void maze_solver::Algorithm::AddNeighbour(std::array<int, 2> neighbour)
{
    if (!IsCellVisited(neighbour))
    {
        this->move_history.push(neighbour);
        this->next_cell = neighbour;
        this->neighbor_added = true;
    }
    else
    {
        neighbor_added = false;
    }
}

/* ==================================================================================================== */

void maze_solver::Algorithm::convertMatrixToString()
{
    this->string_maze_map = "[";
    for (int i = 0; i < this->maze_map.size(); ++i)
    {
        this->string_maze_map += "[";
        for (int j = 0; j < this->maze_map[i].size(); ++j)
        {
            int curr_num = this->maze_map[i][j];
            this->string_maze_map += std::to_string(curr_num);
            this->string_maze_map += ", ";
        }
        this->string_maze_map.pop_back();
        this->string_maze_map.pop_back();
        this->string_maze_map += "],";
    }
    this->string_maze_map.pop_back();
    this->string_maze_map += "]";
}

/* ==================================================================================================== */

void maze_solver::Algorithm::sendMazeMap()
{
    const std::string api_matrix_send{"http://127.0.0.1:8801/api/v1/matrix/send?token=" + this->robot.token};
    this->convertMatrixToString();
    cpr::Response r = cpr::Post(cpr::Url{api_matrix_send},
                                cpr::Body{this->string_maze_map},
                                cpr::Header{{"Content-Type", "application/json"}});
    std::cout << string_maze_map << std::endl;                        
    std::cout << r.text << std::endl;
}

/* ==================================================================================================== */

int maze_solver::Algorithm::cellType()
{
    int distance_wall = 70;

    if (this->robot.getRotationSensorsData().front_distance > distance_wall &&
        this->robot.getRotationSensorsData().right_side_distance > distance_wall &&
        this->robot.getRotationSensorsData().left_side_distance > distance_wall &&
        this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 0;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall &&
             this->robot.getRotationSensorsData().right_side_distance > distance_wall &&
             this->robot.getRotationSensorsData().left_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 1;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall &&
             this->robot.getRotationSensorsData().right_side_distance > distance_wall &&
             this->robot.getRotationSensorsData().left_side_distance > distance_wall &&
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 2;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().left_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 3;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 4;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance < distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 5;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().left_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 6;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall &&
             this->robot.getRotationSensorsData().right_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().left_side_distance > distance_wall &&
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 7;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall &&
             this->robot.getRotationSensorsData().right_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 8;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance < distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 9;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall &&
             this->robot.getRotationSensorsData().right_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 10;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().left_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 11;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance < distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance < distance_wall &&
             this->robot.getRotationSensorsData().back_distance > distance_wall)
    {
        return 12;
    }
    else if (this->robot.getRotationSensorsData().front_distance < distance_wall &&
             this->robot.getRotationSensorsData().right_side_distance > distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance < distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 13;
    }
    else if (this->robot.getRotationSensorsData().front_distance > distance_wall && 
             this->robot.getRotationSensorsData().right_side_distance < distance_wall && 
             this->robot.getRotationSensorsData().left_side_distance < distance_wall && 
             this->robot.getRotationSensorsData().back_distance < distance_wall)
    {
        return 14;
    }

    return 15;
}

/* ==================================================================================================== */

void maze_solver::Algorithm::DFSAlgorithm()
{
    this->move_history.push(this->current_cell);
    int count_visited_cells = 1;
    this->robot.updateSensorsData();
    this->robot.rotationCurrentSensorsData();
    this->maze_map.at(this->current_cell.at(0)).at(this->current_cell.at(1)) = this->cellType();

    while (count_visited_cells < 257)
    {
        FindNeighbours();
        if(this->neighbor_added)
        {
            count_visited_cells++;
            this->visited_cell.at(this->current_cell.at(0)).at(this->current_cell.at(1)) = true;
            this->Navigate(this->next_cell);
        }
        else
        {
            this->move_history.pop();
            this->Navigate(this->move_history.top());
        }

        current_cell = move_history.top();
        this->robot.updateSensorsData();
        this->robot.updateRobotDirection();
        this->robot.rotationCurrentSensorsData();
        this->maze_map.at(current_cell.at(0)).at(current_cell.at(1)) = this->cellType();
    }
}

/* ==================================================================================================== */

void maze_solver::Algorithm::Navigate(std::array<int, 2> next_cell)
{
    int x_cell{}, y_cell{};
    char direction_togo{};

    x_cell = next_cell[0] - this->current_cell[0];
    y_cell = next_cell[1] - this->current_cell[1];

    if (x_cell == -1 && y_cell == 0)
        direction_togo = 'N';
    else if (x_cell == 1 && y_cell == 0)
        direction_togo = 'S';
    else if (x_cell == 0 && y_cell == -1)
        direction_togo = 'W';
    else if (x_cell == 0 && y_cell == 1)
        direction_togo = 'E';

    if (this->robot.getCurrentRobotDirection() == 'N')
    {
        if (direction_togo == 'N')
        {
            if (this->robot.getCurrentSensorsData().front_distance > 70)
                this->robot.MoveForward();
        }
        else if (direction_togo == 'S')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward();
        }
        else if (direction_togo == 'E')
        {
            if (this->robot.getCurrentSensorsData().right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward();
            }
        }
        else if (direction_togo == 'W')
        {
            if (this->robot.getCurrentSensorsData().left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward();
            }
        }
    }
    else if (this->robot.getCurrentRobotDirection() == 'S')
    {
        if (direction_togo == 'N')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward();
        }
        else if (direction_togo == 'S')
        {
            if (this->robot.getCurrentSensorsData().front_distance > 70)
                this->robot.MoveForward();
        }
        else if (direction_togo == 'E')
        {
            if (this->robot.getCurrentSensorsData().left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward();
            }
        }
        else if (direction_togo == 'W')
        {
            if (this->robot.getCurrentSensorsData().right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward();
            }
        }
    }
    else if (this->robot.getCurrentRobotDirection() == 'E')
    {
        if (direction_togo == 'N')
        {
            if (this->robot.getCurrentSensorsData().left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward();
            }
        }
        else if (direction_togo == 'S')
        {
            if (this->robot.getCurrentSensorsData().right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward();
            }
        }
        else if (direction_togo == 'E')
        {
            if (this->robot.getCurrentSensorsData().front_distance > 70)
                this->robot.MoveForward();
        }
        else if (direction_togo == 'W')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward();
        }
    }
    else if (this->robot.getCurrentRobotDirection() == 'W')
    {
        if (direction_togo == 'N')
        {
            if (this->robot.getCurrentSensorsData().right_side_distance > 70)
            {
                this->robot.TurnRight();
                this->robot.MoveForward();
            }
        }
        else if (direction_togo == 'S')
        {
            if (this->robot.getCurrentSensorsData().left_side_distance > 70)
            {
                this->robot.TurnLeft();
                this->robot.MoveForward();
            }
        }
        else if (direction_togo == 'E')
        {
            this->robot.TurnLeft();
            this->robot.TurnLeft();
            this->robot.MoveForward();
        }
        else if (direction_togo == 'W')
        {
            if (this->robot.getCurrentSensorsData().front_distance > 70)
                this->robot.MoveForward();
        }
    }
}