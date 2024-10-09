#include <cpr/cpr.h>
#include <iostream>
#include <algorithm>

std::vector<double> sensorData(std::vector<std::string> &params);
void Forward(int n);
void Backward(int n);
void Left(int n);
void Right(int n);
int cellType(std::vector<double>& vals);
std::vector<std::vector<int>> matrixlab(16,std::vector<int>(16,-1));
std::vector<std::vector<bool>> visited(16,std::vector<bool>(16));
void explore(int x, int y, std::vector<double>& vals);

int main()
{
    std::vector<std::string> params = {
        "front_distance",      // 0
        "right_side_distance", // 1
        "left_side_distance",  // 2
        "back_distance",       // 3
        "left_45_distance",    // 4
        "right_45_distance",   // 5
        "rotation_pitch",      // 6
        "rotation_yaw",        // 7
        "rotation_roll",       // 8
        "down_x_offset",       // 9
        "down_y_offset"        // 10
    };
 
    auto sensor_data = sensorData(params);
    std::cout << cellType(sensor_data) << std::endl;
    Right(1);
}
void explore(int x, int y, std::vector<double>& vals) {
    if (visited[x][y]) return;
    matrixlab[x][y] = cellType(vals);
    double front = vals[0], right = vals[1], left = vals[2], back = vals[3];

}

std::vector<double> sensorData(std::vector<std::string> &params)
{
    auto r = cpr::Get(cpr::Url{"http://127.0.0.1:8801/api/v1/robot-cells/sensor-data?token=1234"});
    std::string sensData = r.text;
    std::vector<double> vals(params.size());
    for (int i = 0; i < params.size() - 1; ++i)
    {
        int beg = sensData.find(params[i]) + params[i].size() + 2;
        int end = sensData.find(params[i + 1]) - 2;
        std::string res = sensData.substr(beg, end - beg);
        vals[i] = std::stod(res);
    }
    int lastbeg = sensData.find(params[params.size() - 1]) + params[params.size() - 1].size() + 2;
    int lastend = sensData.find("}") - 1;
    std::string lastres = sensData.substr(lastbeg, lastend - lastbeg);
    vals[vals.size() - 1] = std::stod(lastres);
    for (int i = 0; i < params.size(); ++i)
    {
        std::cout << params[i] << ": " << vals[i] << std::endl;
    }
    return vals;
}

void Forward(int n){
    while(n--){
        cpr::Response r = cpr::Post(cpr::Url{"http://127.0.0.1:8801/api/v1/robot-cells/forward?token=1234"},
                    cpr::Body{""},
                    cpr::Header{{"accept", "application/json"}});

        std::cout << r.text << std::endl;
    }
}
void Backward(int n){
    while(n--){
        cpr::Response r = cpr::Post(cpr::Url{"http://127.0.0.1:8801/api/v1/robot-cells/backward?token=1234"},
                    cpr::Body{""},
                    cpr::Header{{"accept", "application/json"}});

        std::cout << r.text << std::endl;   
    }
}
void Right(int n){
    while(n--){
        cpr::Response r = cpr::Post(cpr::Url{"http://127.0.0.1:8801/api/v1/robot-cells/right?token=1234"},
                    cpr::Body{""},
                    cpr::Header{{"accept", "application/json"}});

        std::cout << r.text << std::endl;   
    }
}
void Left(int n){
    while(n--){
        cpr::Response r = cpr::Post(cpr::Url{"http://127.0.0.1:8801/api/v1/robot-cells/left?token=1234"},
                    cpr::Body{""},
                    cpr::Header{{"accept", "application/json"}});

        std::cout << r.text << std::endl;   
    }
}

int cellType(std::vector<double>& vals){
    double front = vals[0], right = vals[1], left = vals[2], back = vals[3];
    double yaw = vals[7];
    double mem;
    if(yaw>85 && yaw < 95){
        mem = front;
        front = left;
        left = back;
        back = right;
        right = mem;
    }else if(yaw>-95 && yaw < -85){
        mem = front;
        front = right;
        right = back;
        back = left;
        left = mem;
    }else if(yaw>175 && yaw < 185){
        std::swap(right,left);
        std::swap(back,front);
    }
    
    int tres = 70;
    if(front < tres || right < tres || left < tres || back < tres){
        if(front<tres){
            if(right<tres){
                if(left<tres){
                    if(back<tres) return 15;
                    else return 12;
                }
                else if(back<tres) return 11;
                else return 7;
            }
            else if(left<tres){
                if(back<tres) return 13;
                else return 8;
            }
            else if(back<tres) return 10;
            else return 2;
        }
        else if(left<tres){

            if(right<tres){
                if(back<tres) return 14;
                else return 9;
            }
            else if(back<tres) return 5;
            else return 1;
        }
        else if(right<tres){
            if(back<tres) return 6;
            else return 3;
        }
        else if(back<tres) return 4;
    }else return 0;

    return -1;
}