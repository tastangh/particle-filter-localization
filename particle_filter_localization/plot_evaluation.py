import pandas as pd
import matplotlib.pyplot as plt

# CSV dosyasını oku
df = pd.read_csv("../../evaluation_log.csv")

# Ortalama ve standart sapmalar
pos_mean = df["pos_error"].mean()
pos_std = df["pos_error"].std()
yaw_mean = df["yaw_error"].mean()
yaw_std = df["yaw_error"].std()

print(f"Pozisyon Hatası Ortalaması: {pos_mean:.2f} m")
print(f"Pozisyon Hatası Std: {pos_std:.2f} m")
print(f"Yaw Hatası Ortalaması: {yaw_mean:.2f} rad")
print(f"Yaw Hatası Std: {yaw_std:.2f} rad")

# Hataları çiz
plt.figure(figsize=(10, 5))
plt.plot(df['time'].values, df['pos_error'].values, label='Pozisyon Hatası (m)')
plt.plot(df['time'].values, df['yaw_error'].values, label='Yaw Hatası (rad)')
plt.xlabel('Zaman (s)')
plt.ylabel('Hata')
plt.title('Göreli Konum ve Yönelim Hataları')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Gerçek ve tahmin edilen konumu karşılaştır
plt.figure(figsize=(8, 8))
plt.plot(df['gt_x'].values, df['gt_y'].values, '--', label='Gerçek Konum')
plt.plot(df['est_x'].values, df['est_y'].values, label='Tahmin Konum')
plt.xlabel("X (m)")
plt.ylabel("Y (m)")
plt.title("Mutlak Konum Karşılaştırması")
plt.legend()
plt.grid(True)
plt.axis("equal")
plt.tight_layout()
plt.show()
