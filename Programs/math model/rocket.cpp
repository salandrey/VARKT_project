#include <iostream>
#include <cmath>
#include <vector>
class Stage {
    double m_dry_mass, m_dot, m_burn_duration;
    double m_fuel_mass;
    double m_isp_sea, m_isp_vacuum;
    public:
    Stage(double dry_mass, double dot, double burn_duration, double isp_sea, double isp_vacuum): m_dry_mass(dry_mass), m_fuel_mass(m_dot*m_burn_duration), m_dot(dot), m_burn_duration(burn_duration), m_isp_sea(isp_sea), m_isp_vacuum(isp_vacuum){}
    double getFuelMass() const {
        return m_fuel_mass;
    }
    double getDot() const {
        return m_dot;
    }
    double getDryMass() const {
        return m_dry_mass;
    }
    double getBurnDuration() const {
        return m_burn_duration;
    }
    double getIspSea() const {
        return m_isp_sea;
    }
    double getIspVacuum() const {
        return m_isp_vacuum;
    }
};
class Rocket {
    double m_x, m_y, m_vx, m_vy, m_t;
    int i = 0;
    double m_current_pitch = M_PI/2;
    int count_of_flights_into_the_space = 0;
    const double good_mass = 19.0;
    bool engine_cutoff = false;
    double m_work_time_of_stages = 0;
    double m_mass_of_worked_stages = 0;
    const double m_r{600000.0}, m_GM{3.5316e12};
    std::vector <Stage> stages {Stage(1427.0, 21.53, 113.81, 254, 270), Stage (344, 6.373, 47.1, 265, 320), Stage (46, 0.6474, 308.93, 80, 315)};
    const double m_mass = stages[0].getDryMass() + stages[0].getFuelMass() + stages[1].getDryMass() + stages[1].getFuelMass() + stages[2].getDryMass() + stages[2].getFuelMass() + good_mass;
    public:
    Rocket(double x, double y): m_x(x), m_y(y), m_vx(175.0), m_vy(0.0), m_t(0) {}
    double get_mass() const {
         if (!engine_cutoff && i < static_cast<int>(stages.size())) {
            return m_mass - m_mass_of_worked_stages - stages[i].getDot() * (m_t - m_work_time_of_stages);
        }  else {
            return good_mass;
        }
    }
    double getSpeed() const {
        return std::sqrt(m_vx * m_vx + m_vy * m_vy);
    }
    void check_engine_cutoff(double v_tang, double v_circ) {
        if (v_tang >= v_circ * 0.9999) {
            std::cout << "Отсечка: t=" << m_t << " h=" << getHeight() << " v_tang=" << v_tang << " v_circ=" << v_circ << " m=" << get_mass() << " кг. Общая скорость - " << getSpeed() << " м/с\n";
            engine_cutoff = true;
            i = static_cast<int>(stages.size());
        }
    }
    void stage_work() {
        if (i < stages.size() and stages[i].getBurnDuration() + m_work_time_of_stages < m_t){
            m_work_time_of_stages += stages[i].getBurnDuration();
            m_mass_of_worked_stages += stages[i].getDryMass();
            m_mass_of_worked_stages += stages[i].getFuelMass();
            i++;
        }
    }
    double getX() const {
        return m_x;
    }
    double getY() const {
        return m_y;
    }
    double get_mdot() const{ //m_dot - расход топлива, а кста thrust - тяга
        double r = std::sqrt(m_x*m_x + m_y*m_y);
        double theta  = std::atan2(m_x, m_y);
        double v_tang = m_vx * std::cos(theta) - m_vy * std::sin(theta);
        double v_circ = std::sqrt(m_GM / r);
        if (engine_cutoff) {
            return 0.0;
        }
        if (v_tang >= v_circ * 0.9999) {
            return 0.0;
        } else if (i >= static_cast<int>(stages.size())) {
            return 0.0;
        }
        return stages[i].getDot();
    }
    double get_tangage_angle(double dt) {
        static int x = 0;
        double h     = getHeight();
        double theta = std::atan2(m_x, m_y);
        double v_rad = m_vx * std::sin(theta) + m_vy * std::cos(theta);
        double target;
        if (h < 19000) {
            target = M_PI / 2;
        } else if (h < 52000) {
            double t = (h - 19000) / 57000.0;
            target = M_PI / 2 * (1.0 - t);
        } else if (v_rad > 15.0 && h > 52000) {
            target = -M_PI / 180 * 15.5;
        } else {
            target = 0.0;
        }
        const double MAX_RATE  = 10.0 * M_PI / 180.0;
        const double max_delta = MAX_RATE * dt;
        if (m_current_pitch < 0 && x ==0) {
            std::cout << "Летим под отрицательным углом с момента t = " << m_t << '\n';
            x++;
        }
        if (m_current_pitch == 0 && x ==1) {
            std::cout << "Перестали лететь под отрицательным углом на моменте t = " << m_t << '\n';
            x++;
        }
        double delta = target - m_current_pitch;
        if (delta >  max_delta) delta =  max_delta;
        if (delta < -max_delta) delta = -max_delta;
        m_current_pitch += delta;
        return m_current_pitch;
    }
    double getTime() const {
        return m_t;
    }
    double getHeight() const{
        return std::sqrt(m_x * m_x + m_y * m_y) - m_r;
    }
        double get_isp(double h) const {
        if (i >= stages.size()) return 0.0;
        // Интерполяция Isp между уровнем моря и вакуумом по экспоненте
        double pressure_ratio = std::exp(-h / 5600.0); 
        return stages[i].getIspVacuum() + (stages[i].getIspSea() - stages[i].getIspVacuum()) * pressure_ratio;
    }
    bool updatePosition(double dt);
};