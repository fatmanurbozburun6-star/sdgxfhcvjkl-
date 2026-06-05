```markdown
   # Engelden Kaçan Robot Ödevi (ROS Noetic)

   Bu proje, bir mobil robotun (TurtleBot3) üzerindeki Lidar sensörünü kullanarak çevresindeki engelleri algılamasını ve bu engellere çarpmadan otonom bir şekilde hareket etmesini (engelden kaçma algoritması) sağlar.

   ## Proje Yapısı ve Dosyalar
   * `scripts/engelden_kacma.py`: Lidar verilerini (`/scan`) dinleyen ve robota hız komutları (`/cmd_vel`) gönderen ana Python kodumuzdur.

   ## Çalıştırma Adımları

   1. Bu paketi `catkin_ws/src/` dizinine klonlayın veya taşıyın.
   2. Çalışma alanını derleyin ve kaynak dosyayı yenileyin:
```bash
      cd ~/catkin_ws
      catkin_make
      source devel/setup.bash
      ```
   3. Python dosyasına çalıştırma izni verin (eğer verilmediyse):
```bash
      chmod +x src/odev_engelden_kacma/scripts/engelden_kacma.py
      ```
   4. İlk olarak Gazebo simülasyon ortamınızı başlatın.
   5. Ardından bu engelden kaçma düğümünü (node) çalıştırın:
```bash
      rosrun odev_engelden_kacma engelden_kacma.py
      ```
