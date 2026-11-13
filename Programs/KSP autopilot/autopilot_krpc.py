import krpc
import time
import math
import csv

def run_autopilot():
    conn = krpc.connect(name='Avangard_1_Autopilot')
    vessel = conn.space_center.active_vessel

    # Константы Кербина из C++ модели
    R_KERBIN = 600000.0
    GM_KERBIN = 3.5316e12

    # Тайминги работы ступеней из C++:
    # S4: 113.81 с | S3: 47.10 с (суммарно 160.91 с) | S2: до отсечки (макс +308.93 с)
    STAGE_1_END = 113.81
    STAGE_2_END = 113.81 + 47.10  # 160.91 с

    # Потоки данных из KSP
    ut = conn.add_stream(getattr, conn.space_center, 'ut')
    altitude = conn.add_stream(getattr, vessel.flight(), 'mean_altitude')
    mass = conn.add_stream(getattr, vessel, 'mass')

    # Инерциальная (невращающаяся) система отсчета Кербина — полный аналог осей (x, y) в C++
    orbital_frame = vessel.orbit.body.non_rotating_reference_frame
    orb_speed = conn.add_stream(getattr, vessel.flight(orbital_frame), 'speed')

    # Поверхностная система отсчета для получения вертикальной (v_rad) и горизонтальной (v_tang) скоростей
    surf_frame = vessel.orbit.body.reference_frame
    v_rad_stream = conn.add_stream(getattr, vessel.flight(surf_frame), 'vertical_speed')
    v_surf_horiz = conn.add_stream(getattr, vessel.flight(surf_frame), 'horizontal_speed')

    # Настройка автопилота перед стартом
    vessel.control.sas = False
    vessel.control.rcs = False
    vessel.control.throttle = 1.0

    current_pitch = 90.0  # В градусах (M_PI / 2)
    MAX_RATE = 10.0       # 10 град/с, как в C++

    vessel.auto_pilot.engage()
    vessel.auto_pilot.target_pitch_and_heading(current_pitch, 90)  # 90 — старт строго на восток

    print("Ракета на стартовом столе. Пуск!")
    vessel.control.activate_next_stage()  # Зажигание S4 (первой ступени)

    start_time = ut()
    dt = 0.1
    stage_idx = 0
    engine_cutoff = False

    # Если нужно писать в CSV каждую 0.1 с (для гладких графиков) или раз в 10 с, как в C++
    with open('ksp_telemetry.csv', mode='w', newline='') as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(['time', 'height', 'mass', 'speed'])

        while True:
            loop_start = time.time()
            m_t = ut() - start_time
            h = altitude()
            m = mass()
            v_orb = orb_speed()
            v_rad = v_rad_stream()

            # Оценка орбитальной тангенциальной скорости v_tang и первой космической v_circ
            r = R_KERBIN + h
            v_circ = math.sqrt(GM_KERBIN / r)
            v_tang_sq = max(0.0, v_orb**2 - v_rad**2)
            v_tang = math.sqrt(v_tang_sq)

            # 1. Логика разделения ступеней (stage_work + фоллбэк по фактической тяге в KSP)
            if not engine_cutoff:
                if stage_idx == 0 and (m_t > STAGE_1_END or vessel.available_thrust < 1.0):
                    print(f"[t={m_t:.1f} с] Отделение 1-й ступени (S4) и запуск 2-й (S3)")
                    vessel.control.activate_next_stage()
                    time.sleep(0.05)
                    # Если в KSP отделитель и двигатель разнесены на разные нажатия пробела:
                    if vessel.available_thrust < 1.0:
                        vessel.control.activate_next_stage()
                    stage_idx = 1

                elif stage_idx == 1 and (m_t > STAGE_2_END or vessel.available_thrust < 1.0):
                    print(f"[t={m_t:.1f} с] Отделение 2-й ступени (S3) и запуск 3-й (S2 «Муравей»)")
                    vessel.control.activate_next_stage()
                    time.sleep(0.05)
                    if vessel.available_thrust < 1.0:
                        vessel.control.activate_next_stage()
                    stage_idx = 2

            # 2. Проверка условия отсечки двигателя (check_engine_cutoff)
            if not engine_cutoff and v_tang >= v_circ * 0.9999:
                vessel.control.throttle = 0.0
                # Принудительно глушим все двигатели, чтобы они ни при каких условиях не включились снова
                for engine in vessel.parts.engines:
                    engine.active = False
                engine_cutoff = True
                print(
                    f"Отсечка: t={m_t:.1f} h={h:.1f} v_tang={v_tang:.2f} "
                    f"v_circ={v_circ:.2f} m={m:.2f} кг. Общая скорость - {v_orb:.2f} м/с"
                )
                # Отстреливаем 3-ю ступень («Муравей»), оставляя чистый спутник (19 кг)
                time.sleep(0.2)
                vessel.control.activate_next_stage()

            # 3. Закон управления тангажом (get_tangage_angle)
            if not engine_cutoff:
                if h < 19000.0:
                    target_pitch = 90.0
                elif h < 52000.0:
                    t_norm = (h - 19000.0) / 57000.0
                    target_pitch = 90.0 * (1.0 - t_norm)
                elif v_rad > 15.0 and h > 52000.0:
                    target_pitch = -15.5
                else:
                    target_pitch = 0.0

                max_delta = MAX_RATE * dt
                delta = target_pitch - current_pitch
                if delta > max_delta:
                    delta = max_delta
                if delta < -max_delta:
                    delta = -max_delta

                current_pitch += delta
                vessel.auto_pilot.target_pitch_and_heading(current_pitch, 90)
            else:
                # На каждом шаге после отсечки гарантируем нулевую тягу!
                vessel.control.throttle = 0.0
                vessel.auto_pilot.target_pitch_and_heading(0.0, 90)
            # 4. Запись телеметрии в CSV
            writer.writerow([f"{m_t:.3f}", f"{h:.3f}", f"{m:.3f}", f"{v_orb:.3f}"])
            csv_file.flush()

            # Проверка на падение или окончание записи
            if h < 0 and m_t > 5.0:
                print(f"Ракета упала на Кербин на {m_t:.1f} секунде!")
                break

            # Останавливаем запись после выхода на орбиту (или по таймеру t_end)
            if m_t >= 7200.0:
                print("Двухчасовая симуляция завершена! Логи сохранены в ksp_telemetry.csv")
                break

            # Выдерживаем шаг цикла ~0.1 с
            elapsed = time.time() - loop_start
            if elapsed < dt:
                time.sleep(dt - elapsed)

if __name__ == '__main__':
    run_autopilot()