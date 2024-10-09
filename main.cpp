#include <fmt/core.h>
#include <cpr/cpr.h>

int main()
{
    cpr::Post(cpr::Url{"http://127.0.0.1:8801/api/v1/robot-cells/right?token=4326f9a3-1628-4884-90cb-c1b93890d61a8ffcd3c7-3620-4a0d-90b3-3dbe0b1c6267"});
    return 0;
}