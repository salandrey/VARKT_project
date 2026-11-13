import csv
import matplotlib.pyplot as plt


def read_csv(filename):
  t, h, m, v = [], [], [], []
  with open(filename, 'r', encoding='utf-8') as f:
    reader = csv.DictReader(f)
    for row in reader:
      t.append(float(row['time']))
      h.append(float(row['height']) / 1000.0)  # Переводим в км для наглядности
      m.append(float(row['mass']))
      v.append(float(row['speed']))
  return t, h, m, v


t_cpp, h_cpp, m_cpp, v_cpp = read_csv('cpp_telemetry.csv')

# Когда появятся логи из KSP, раскомментируй следующие две строки:
t_ksp, h_ksp, m_ksp, v_ksp = read_csv('ksp_telemetry.csv')

fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 12), sharex=True)

# 1. График высоты
ax1.plot(t_cpp, h_cpp, label='Мат. модель (C++)', color='#1f77b4', linewidth=2)
ax1.plot(t_ksp, h_ksp, label='KSP (Автопилот)', color='#ff7f0e', linestyle='--', linewidth=2)
ax1.set_ylabel('Высота (км)')
ax1.set_title('Сравнение телеметрии полета: Мат. модель vs KSP')
ax1.grid(True, linestyle=':', alpha=0.7)
ax1.legend()

# 2. График скорости
ax2.plot(t_cpp, v_cpp, label='Мат. модель (C++)', color='#2ca02c', linewidth=2)
ax2.plot(t_ksp, v_ksp, label='KSP (Автопилот)', color='#d62728', linestyle='--', linewidth=2)
ax2.set_ylabel('Скорость (м/с)')
ax2.grid(True, linestyle=':', alpha=0.7)
ax2.legend()

# 3. График массы
ax3.plot(t_cpp, m_cpp, label='Мат. модель (C++)', color='#9467bd', linewidth=2)
ax3.plot(t_ksp, m_ksp, label='KSP (Автопилот)', color='#8c564b', linestyle='--', linewidth=2)
ax3.set_xlabel('Время полета (с)')
ax3.set_ylabel('Масса (кг)')
ax3.grid(True, linestyle=':', alpha=0.7)
ax3.legend()

plt.tight_layout()
plt.savefig('comparison_graphs.png', dpi=300)
plt.show()