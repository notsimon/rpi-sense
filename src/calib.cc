#include "led_matrix.h"
#include "lsp25h.h"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <vector>

void compute_var(const std::vector<float>& data, float& mean, float& var) {
    mean = 0;
    for (auto e : data) {
        mean += e;
    }
    mean /= data.size();

    var = 0;
    for (auto e : data) {
        var += (e - mean)*(e - mean);
    }
    var /= data.size();
}

int main(void)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;

    const Duration dt = std::chrono::milliseconds(50);

    Lsp25h lsp25h;

    std::vector<float> pressure;
    std::vector<float> temp;

    for (size_t i = 0;; i++) {
        auto start = Clock::now();

        pressure.push_back(lsp25h.read_pressure());
        temp.push_back(lsp25h.read_temp());

        if (i % 20 == 0) {
            float pressure_mean, pressure_var, temp_mean, temp_var;

            compute_var(pressure, pressure_mean, pressure_var);
            std::cout << "PRESSURE: " << std::fixed
                << "mean =" << std::setprecision(3) << pressure_mean
                << ", var = " << std::setprecision(6) << pressure_var << std::endl;

            compute_var(temp, temp_mean, temp_var);
            std::cout << "TEMPERATURE: " << std::fixed
                << "mean =" << std::setprecision(3) << temp_mean
                << ", var = " << std::setprecision(6) << temp_var << std::endl;
        }

        auto stop = Clock::now();
        auto remaining = dt - (stop - start);

        if (remaining >= Duration(0)) {
            std::this_thread::sleep_for(remaining);
        } else {
            std::cerr << "OVERRUN!" << std::endl;
        }
    }

    return 0;
}
