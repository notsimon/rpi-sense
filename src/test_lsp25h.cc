#include "led_matrix.h"
#include "lsp25h.h"
#include <cstdio>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

int main(void)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;

    const Duration dt = std::chrono::milliseconds(50);

    Lsp25h lsp25h;

    while (1) {
        auto start = Clock::now();

        double pressure = lsp25h.read_pressure();
        double temp = lsp25h.read_temp();

        std::cout << std::fixed << std::setprecision(3) << pressure
            << ", " << std::setprecision(2) << temp << std::endl;

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
