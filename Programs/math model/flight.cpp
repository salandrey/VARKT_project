#include <iostream>
#include "rocket.cpp"
#include <cmath>
struct FlightData {
    double m_time;
    double m_height;
    double m_mass;
    double m_speed;
    double m_acceleration;
    double m_x;
    double m_y;
    FlightData(double time, double height, double mass, double speed, double x, double y): m_time(time), m_height(height), m_mass(mass), m_speed(speed), m_x(x), m_y(y){}
};
bool Rocket::updatePosition (double dt) {
    double r = std::sqrt(m_x*m_x + m_y*m_y);
    const double rho0 = 1.225;
    const double H = 5600.0;
    const double Cd = 0.2;
    const double S = 4.15;
    const double g_standard = 9.80665;
    double theta  = std::atan2(m_x, m_y);
    double v_tang = m_vx * std::cos(theta) - m_vy * std::sin(theta);
    double v_circ = std::sqrt(m_GM / r);
    stage_work();
    if (!get_mdot() && !engine_cutoff) {
        check_engine_cutoff(v_tang, v_circ);
    }
    double height = getHeight();
    if (height < 0 && m_t > 0.1) {
        std::cout << "Ракета упала на Кербин на " << m_t << " секунде!" << " Координаты точки падения : " << m_x << ", " << m_y << std::endl;
        return false;
    } 
    double v = getSpeed();
    double alpha = get_tangage_angle(dt);
    double thrust_angle_global = alpha - theta;
    double current_mass = get_mass();
    double mdot = get_mdot();
    double rho = (height < 70000.0) ? rho0 * std::exp(-height / H) : 0.0;
    double Isp = get_isp(height);
    double F_thrust = Isp * g_standard * mdot;
    double F_thrust_x = F_thrust * std::cos(thrust_angle_global);
    double F_thrust_y = F_thrust * std::sin(thrust_angle_global);
    const double omega = 175.0 / 600000.0; 
    double v_atm_x = omega * m_y;
    double v_atm_y = -omega * m_x;
    double v_rel_x = m_vx - v_atm_x;
    double v_rel_y = m_vy - v_atm_y;
    double v_rel = std::sqrt(v_rel_x * v_rel_x + v_rel_y * v_rel_y);
    double F_drag_x = 0.0;
    double F_drag_y = 0.0;
    if (rho > 0.0 && v_rel > 0.001) {
        double drag_factor = 0.5 * rho * v_rel * Cd * S;
        F_drag_x = -drag_factor * v_rel_x;
        F_drag_y = -drag_factor * v_rel_y;
    }
    double F_grav_x = -(m_GM * current_mass * m_x) / (r * r * r);
    double F_grav_y = -(m_GM * current_mass * m_y) / (r * r * r);
    double ax = (F_thrust_x + F_drag_x + F_grav_x) / current_mass;
    double ay = (F_thrust_y + F_drag_y + F_grav_y) / current_mass;
    m_vx += ax * dt;
    m_vy += ay * dt;
    m_x += m_vx * dt;
    m_y += m_vy * dt;
    m_t += dt;
    return true;
}