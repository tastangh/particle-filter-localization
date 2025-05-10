import pandas as pd
import matplotlib.pyplot as plt

# CSV dosyasını oku
df = pd.read_csv("../../evaluation_log.csv")

# Zaman ve hataları çiz
plt.figure(figsize=(10, 5))
plt.plot(df['time'].values, df['pos_error'].values, label='Pozisyon Hatası (m)')
plt.plot(df['time'].values, df['yaw_error'].values, label='Yaw Hatası (rad)')
plt.xlabel('Zaman (s)')
plt.ylabel('Hata')
plt.title('Konum ve Yönelim Hataları')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()
