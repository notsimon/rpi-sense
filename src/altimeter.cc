#include "led_matrix.h"
#include "lsp25h.h"
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <chrono>
#include <thread>
#include <iostream>
#include <iomanip>
#include <limits>
#include <cmath>

int main(void)
{
    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;

    const Duration dt = std::chrono::milliseconds(100);

    Lsp25h lsp25h;
    LedMatrix led_matrix;

    const time_t start_t = time(NULL);
    double pressure_zero = std::numeric_limits<double>::infinity();
    double altitude_zero = std::numeric_limits<double>::infinity();

    while (1) {
        auto start = Clock::now();

        double pressure = lsp25h.read_pressure();
        double temp = lsp25h.read_temp();

        // US Standard Atmosphere formula, see section 6.4 of the LSP25H app note
        double altitude = (1.0 - pow(pressure/1013.25, 0.190284))*145366.45/3.280839895;

        if (std::isfinite(altitude_zero)) {
            altitude = std::max(0.0, altitude - altitude_zero);
        }

        // Hypsometric equation: https://en.wikipedia.org/wiki/Hypsometric_equation
        double altitude_hypso = log(pressure_zero / pressure) * 8.3144621 * (temp + 273.15) / (9.80665 * 0.028965338);

        std::cout << std::fixed << "P = " << std::setprecision(2) << pressure << " hPa; "
            << "T = " << std::setprecision(1) << temp << " \u00B0C; "
            << "Alt(P) = " << std::setprecision(1) << altitude << " m; "
            << "Alt(P, T) = " << std::setprecision(1) << altitude_hypso << " m" << std::endl;

        // show current level on the led screen
        led_matrix.set(0, 0, 0);
        const size_t n = std::min<size_t>(std::round(altitude/2.0*LedMatrix::Height), LedMatrix::Height);
        for (size_t x = 0; x < LedMatrix::Width; x++) {
            for (size_t y = 0; y < n; y++) {
                led_matrix.set(x, y, 255, 0, 0);
            }
        }

        // set offset to current value after a few seconds
        if (std::isinf(altitude_zero) && std::isinf(pressure_zero) && time(NULL) - start_t > 5) {
            pressure_zero = pressure;
            altitude_zero = altitude;
            std::cout << "Pressure of reference = " << pressure_zero <<
                ", altitude of reference = " << altitude_zero << std::endl;;
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
