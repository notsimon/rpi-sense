#include "led_matrix.h"
#include "lsp25h.h"
#include "kalman.h"
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>

int main(void)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;
    using std::chrono::duration_cast;

    const Duration dt = std::chrono::milliseconds(100);

    Lsp25h lsp25h;
    LedMatrix led_matrix;
    KalmanFilter filter;

    bool zeroed = false;
    const time_t start_t = time(NULL);

    while (1) {
        auto start = Clock::now();

        float pressure = lsp25h.read_pressure();
        float temp = lsp25h.read_temp();

        if (zeroed) {
            filter(pressure, temp + 273.15, duration_cast<std::chrono::duration<float>>(dt).count());
        }

        float altitude = filter.state().altitude();

        std::cout << std::fixed
            << "P = " << std::setprecision(2) << pressure << " hPa; "
            << "T = " << std::setprecision(1) << temp << " \u00B0C (" << temp + 273.15 << " \u00B0K); "
            << "H = " << std::setprecision(1) << altitude << " m" << std::endl;

        // show current level on the led screen
        led_matrix.set(0, 0, 0);
        const size_t n = std::min<size_t>(std::round(altitude/2.0*LedMatrix::Height), LedMatrix::Height);
        for (size_t x = 0; x < LedMatrix::Width; x++) {
            for (size_t y = 0; y < n; y++) {
                led_matrix.set(x, y, 156 + std::rand() % 100, 0, 0);
            }
        }

        // set offset to current value after a few seconds
        if (!zeroed && time(NULL) - start_t > 2) {
            zeroed = true;
            filter.ref_pressure() = pressure;
            std::cout << "Pressure of reference = " << pressure << std::endl;
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
