#include <mujoco.h>
#include <iostream>
#include <cstdlib>
mjModel* model = nullptr;
mjData* data = nullptr;

int main() {
    char error[1000] = "Could not load model";
    model = mj_loadXML("path/to/model.xml", nullptr, error, 1000);
    if (!model) {
        std::cerr << error << std::endl;
        return 1;
    }

    // Create simulation data
    data = mj_makeData(model);

    // Run simulation loop
    for (int i = 0; i < 1000; i++) {
        mj_step(model, data);
        std::cout << "Time: " << data->time << std::endl;
    }

    // Free resources
    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
