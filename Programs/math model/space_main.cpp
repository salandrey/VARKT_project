#include <iostream>
#include <fstream>
#include "flight.cpp"
#include <vector>
#include <iomanip>
int main() {
    Rocket avangard_1 (0.0, 600000);
    std::cout << "Ракета готова. Симуляция началась!" << std::endl;
    double dt = 0.1;
    double checkpoints_time=0.1;
    double t_end = 7200;
    std::vector <FlightData> telemetry;
    while (avangard_1.getTime() < t_end) {
        if (!(avangard_1.updatePosition(dt))){
            break;
        }
        double current_t = avangard_1.getTime();
        static double next_save_t = 0.0;
        if (current_t >= next_save_t) {
            telemetry.push_back({current_t, avangard_1.getHeight(), avangard_1.get_mass(), avangard_1.getSpeed(), avangard_1.getX(), avangard_1.getY()});
            next_save_t += checkpoints_time;
        }
    }
    std::ofstream csv_file("cpp_telemetry.csv");
    csv_file << "time,height,mass,speed\n";
    csv_file << std::fixed << std::setprecision(3);
    for (const auto& data : telemetry) {
        csv_file << data.m_time << ","
                 << data.m_height << ","
                 << data.m_mass << ","
                 << data.m_speed << "\n";
    }
    csv_file.close();
    std::cout << "Логи сохранены в cpp_telemetry.csv\n";
    std::cout << "Симуляция завершена!" << std::endl;
    std::cout << "Количество чек-поинтов в телеметрии: " << telemetry.size() << std::endl;
    std::cout << "\n--- Отчет телеметрии полета (вывод раз в " << checkpoints_time << " секунд) ---" << std::endl;
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Время (с) | Высота (м) | Масса (кг)  | Скорость (м/с)   |   x          |    y " << std::endl;
    std::cout << "-----------------------------------------------" << std::endl;
    
    for (size_t i = 0; i < telemetry.size(); i += 1) {
        std::cout << std::setw(8) << telemetry[i].m_time << "  | "
                  << std::setw(10) << telemetry[i].m_height << " | "
                  << std::setw(9) << telemetry[i].m_mass << "   | "
                  << std::setw(11) << telemetry[i].m_speed << "      | " << std::setw(12) << telemetry[i].m_x << " | " << std::setw(8) << telemetry[i].m_y << std::endl;
    }

    return 0;
}