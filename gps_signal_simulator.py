#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GPS-имитатор согласно техническому заданию
- Угол руля: -600...+600 градусов
- Скорость: 0-50 км/ч  
- Wheelbase: 2 метра
- Дискретность: 100 мс
- Вывод: 10 Гц (каждые 100 мс)
"""

import math
import time
import threading
from datetime import datetime, timezone


class GPSSimulator:
    """
    GPS-симулятор транспортного средства.
    
    Класс реализует симуляцию движения авто с генерацией
    стандартных NMEA сообщений (GGA, VTG)
    
    Основные возможности:
    - Симуляция движения с учетом угла поворота руля и скорости
    - Модель поворота Аккермана
    - Многопоточная архитектура для независимой работы симуляции и вывода
    - Генерация NMEA сообщений с контрольной суммой
    - Интерактивное управление через командную строку
    - Сброс к исходным настройкам
    
    Параметры симуляции согласно ТЗ:
    - Диапазон углов руля: -600...+600 градусов
    - Диапазон скоростей: 0-50 км/ч
    - База колес: 2 метра
    - Дискретность обновления: 100 мс
    - Частота вывода NMEA: 10 Гц
    - Начальные координаты: 53.262778°N, 50.372778°E
    
    Примеры использования без GUI:
        >>> simulator = GPSSimulator()
        >>> simulator.set_speed(15)  # 15 км/ч
        >>> simulator.set_steering_angle(30)  # поворот на 30 градусов
        >>> simulator.start()  # запуск симуляции
        >>> simulator.get_status()  # проверка состояния
        >>> simulator.reset()  # сброс к исходным настройкам
    
    Генерируемые NMEA сообщения:
    - GGA: координаты, время, качество сигнала
    - VTG: скорость и курс движения
    
    Потокобезопасность:
    Все методы изменения состояния (set_speed, set_steering_angle, reset) 
    защищены мьютексом для безопасной работы в многопоточной среде.
    """
    
    def __init__(self):
        """
        Инициализация GPS-симулятора с параметрами по умолчанию.
        
        Устанавливает начальные координаты, параметры машины
        и создает объекты синхронизации для многопоточной работы.
        """
        # Параметры из ТЗ
        self.wheelbase = 2.0 # База колес [м]
        self.dt = 0.1  # Дискретность обновления [с]
        
        # Начальные GPS координаты из ТЗ
        self.initial_lat = 53.262778
        self.initial_lon = 50.372778
        self.earth_radius = 6378137.0
        
        # Значения по умолчанию для сброса
        self.default_x = 0.0
        self.default_y = 0.0
        self.default_heading = 0.0
        self.default_speed_kmh = 0.0
        self.default_steering_angle = 0.0
        
        # Текущее состояние транспорта
        self.x = self.default_x  # положение в декартовых
        self.y = self.default_y  # координатах [м]
        self.heading = self.default_heading  # курс в радианах (0 = север)
        self.speed_kmh = self.default_speed_kmh  # скорость [км/ч]
        self.steering_angle = self.default_steering_angle  # угол руля [°]
        
        # Управление потоками
        self.running = False   # Флаг работы симулятора
        self.output_thread = None # Поток вывода NMEA сообщений
        self.sim_thread = None  # Поток симуляции движения
        
        # Потокобезопасность
        self.lock = threading.Lock()
        
    def set_speed(self, speed):
        """
        Определение скорости движения транспортного средства.
        
        Args:
            speed (float): Скорость в км/ч (диапазон: 0-50)
            
        Returns:
            bool: True если скорость установлена успешно, False при ошибке
            
        Raises:
            Выводит сообщение об ошибке в консоль при некорректных данных
        """
        try:
            speed = float(speed)
            if 0 <= speed <= 50:
                with self.lock:
                    self.speed_kmh = speed
                print(f"Скорость установлена: {speed} км/ч")
                return True
            else:
                print(f"Ошибка: скорость должна быть от 0 до 50 км/ч (получено: {speed})")
                return False
        except (ValueError, TypeError) as e:
            print(f"Ошибка: введите числовое значение скорости (получено: {speed})")
            return False
    
    def set_steering_angle(self, angle):
        """
        Установка угла поворота руля.
        
        Args:
            angle (float): Угол поворота руля в градусах (диапазон: -600...+600)
                          Отрицательные значения - поворот влево
                          Положительные значения - поворот вправо
                          
        Returns:
            bool: True если угол установлен успешно, False при ошибке

        Raises:
            bool: False если в консоль при некорректных данных
        """
        try:
            angle = float(angle)
            if -600 <= angle <= 600:
                with self.lock:
                    self.steering_angle = angle
                print(f"Угол руля установлен: {angle}°")
                return True
            else:
                print(f"Ошибка: угол руля должен быть от -600 до +600 градусов (получено: {angle})")
                return False
        except (ValueError, TypeError) as e:
            print(f"Ошибка: введите числовое значение угла (получено: {angle})")
            return False
    
    def reset(self):
        """
        Сброс к исходным настройкам.
        
        Возвращает все параметры к значениям по умолчанию:
        - Позиция: (0, 0) в декартовых координатах
        - Курс: 0° (север)
        - Скорость: 0 км/ч
        - Угол руля: 0°

        Returns:
            bool: True если сброс выполнен успешно

        """
        with self.lock:
            self.x = self.default_x
            self.y = self.default_y
            self.heading = self.default_heading
            self.speed_kmh = self.default_speed_kmh
            self.steering_angle = self.default_steering_angle
            
        print("Симулятор сброшен к исходным настройкам:")
        print(f"- Позиция: ({self.default_x}, {self.default_y}) м")
        print(f"- Курс: {math.degrees(self.default_heading)}°")
        print(f"- Скорость: {self.default_speed_kmh} км/ч")
        print(f"- Угол руля: {self.default_steering_angle}°")
        return True
    
    def calculate_wheel_angle(self, steering_angle, transmission_ratio=15.0):
        """
        Преобразование угла руля в угол поворота колес.
        
        Использует передаточное соотношение рулевого управления.
        В качестве значения по умолчанию используется 15:1,
        
        Args:
            steering_angle (float): Угол поворота руля в градусах
            
        Returns:
            float: Угол поворота колес в градусах
        """
        return steering_angle / transmission_ratio
    
    def update_position(self):
        """
        Обновление позиции транспортного средства.
        
        Вычисляет новые координаты и курс на основе текущей скорости, 
        угла поворота руля и времени дискретизации. Использует математически
        корректную модель рулевого управления:
    
        - Радиус поворота: R = L / tan(δ), где L - база колес, δ - угол колес
        - Угловую скорость: ω = v / R
        - Траекторию движения по дуге окружности
        - Прямолинейное движение при малом угле поворота
        """
        with self.lock:
            current_speed = self.speed_kmh
            current_angle = self.steering_angle
            
        if current_speed == 0:
            return
        
        # Перевед скорости из км/ч в м/с
        speed_ms = current_speed / 3.6
        
        # Перевод угла поворота колес из градусов в радианы
        wheel_angle_deg = self.calculate_wheel_angle(current_angle)
        wheel_angle_rad = math.radians(wheel_angle_deg)
        
        with self.lock:
            if abs(wheel_angle_rad) < 0.001:
                # Прямолинейное движение
                dx = speed_ms * math.sin(self.heading) * self.dt
                dy = speed_ms * math.cos(self.heading) * self.dt
                self.x += dx
                self.y += dy
            else:
                # Поведение при повороте, описанное моделью Аккермана 
                turn_radius = self.wheelbase / math.tan(abs(wheel_angle_rad))
                
                # Угловая скорость ω = v / R
                angular_velocity = speed_ms / turn_radius
                
                # Учитываем направление поворота
                if wheel_angle_rad < 0:  # отрицательный угол = поворот влево
                    angular_velocity = -angular_velocity
                    
                # Изменение курса за время dt
                dtheta = angular_velocity * self.dt
                self.heading += dtheta
                
                # Перемещение по дуге
                if abs(dtheta) > 0.001:
                    # Радиус кривизны траектории
                    arc_radius = speed_ms * self.dt / dtheta
                    # Смещение перпендикулярно к начальному направлению
                    dx = arc_radius * (math.sin(self.heading) - math.sin(self.heading - dtheta))
                    dy = arc_radius * (math.cos(self.heading - dtheta) - math.cos(self.heading))
                else:
                    # При очень малом повороте - почти прямолинейное движение
                    dx = speed_ms * math.sin(self.heading - dtheta/2) * self.dt
                    dy = speed_ms * math.cos(self.heading - dtheta/2) * self.dt
                    
                self.x += dx
                self.y += dy
            
            # Нормализация курса (0 до 2π)
            # необходимо для корректного отображения в NMEA сообщениях
            self.heading = self.heading % (2 * math.pi)
    
    def cartesian_to_gps(self, x, y):
        """
        Преобразование положения из декартовых координат в полярные координаты с
        использованием элементарного тригометрического перехода проекции
        для малых расстояний от базовой точки.
        
        Args:
            x (float): X координата в метрах
            y (float): Y координата в метрах
            
        Returns:
            tuple: (широта, долгота) в градусах
        """
        # Простое приближение для малых расстояний
        lat_rad = math.radians(self.initial_lat)
        
        # Смещения в градусах
        lat_offset = y / self.earth_radius * 180.0 / math.pi
        lon_offset = x / (self.earth_radius * math.cos(lat_rad)) * 180.0 / math.pi
        
        latitude = self.initial_lat + lat_offset
        longitude = self.initial_lon + lon_offset
        
        return latitude, longitude
    
    def calculate_checksum(self, sentence):
        """
        Расчет контрольной суммы NMEA сообщения.

        Использует алгоритм XOR для всех символов между '$' и '*'.
        
        Args:
            sentence (str): NMEA сообщение без символов '$' и '*'
            
        Returns:
            str: Контрольная сумма в шестнадцатеричном формате (2 символа)
        """
        checksum = 0
        for char in sentence:
            checksum ^= ord(char)
        return f"{checksum:02X}"
    
    def generate_gga_message(self):
        """
        Генерация NMEA GGA сообщения (положения и качествасигнала).
        
        Returns:
            str: Полное GGA сообщение с контрольной суммой
        """
        with self.lock:
            lat, lon = self.cartesian_to_gps(self.x, self.y)
            
        now = datetime.now(timezone.utc)
        time_str = now.strftime("%H%M%S.%f")[:-3]  # HHMMSS.SSS
        
        # Формат широты DDMM.MMMM
        lat_deg = int(abs(lat))
        lat_min = (abs(lat) - lat_deg) * 60
        lat_str = f"{lat_deg:02d}{lat_min:07.4f}"
        lat_dir = "N" if lat >= 0 else "S"
        
        # Формат долготы DDDMM.MMMM  
        lon_deg = int(abs(lon))
        lon_min = (abs(lon) - lon_deg) * 60
        lon_str = f"{lon_deg:03d}{lon_min:07.4f}"
        lon_dir = "E" if lon >= 0 else "W"
        
        # Составляем GGA сообщение
        gga_data = f"GPGGA,{time_str},{lat_str},{lat_dir},{lon_str},{lon_dir},1,01,1.0,45.0,M,0.0,M,,"
        checksum = self.calculate_checksum(gga_data)
        
        return f"${gga_data}*{checksum}"
    
    def generate_vtg_message(self):
        """
        Генерация NMEA VTG сообщения (скорость и курс).
        
        Returns:
            str: Полное VTG сообщение с контрольной суммой
        """
        with self.lock:
            # Курс в градусах (0-360)
            course_deg = math.degrees(self.heading) % 360
            
            # Скорость в узлах
            speed_knots = self.speed_kmh / 1.852
            current_speed = self.speed_kmh
        
        # Составляем VTG сообщение
        vtg_data = f"GPVTG,{course_deg:.1f},T,,M,{speed_knots:.1f},N,{current_speed:.1f},K,A"
        checksum = self.calculate_checksum(vtg_data)
        
        return f"${vtg_data}*{checksum}"
    
    def output_gps_messages(self):
        """
        Поток вывода GPS сообщений с частотой ≈ 10 Гц.
        
        Генерирует и выводит GGA и VTG сообщения каждые 100 мс.
        Завершается при установке флага self.running в False.
        """
        while self.running:
            try:
                gga_msg = self.generate_gga_message()
                vtg_msg = self.generate_vtg_message()
                
                print(gga_msg)
                print(vtg_msg)
                print()  # Пустая строка между блоками
                
                time.sleep(self.dt)  # 0.1 с = 10 Гц
            except Exception as e:
                print(f"Ошибка вывода GPS сообщений: {e}")
                break
    
    def simulation_loop(self):
        """
        Основной цикл симуляции движения.
        
        Обновляет позицию транспортного средства каждые 0.1 с
        на основе текущих параметров скорости и угла поворота руля.
        """
        while self.running:
            try:
                self.update_position()
                time.sleep(self.dt) 
            except Exception as e:
                print(f"Ошибка в цикле симуляции: {e}")
                break
    
    def start(self):
        """
        Функция запуска симулятора.
        
        Создает и запускает два потока:
        - Поток симуляции для обновления позиции
        - Поток вывода для генерации NMEA сообщений
        
        Returns:
            bool: True если симулятор запущен успешно, False если уже работает
        """
        with self.lock:
            if not self.running:
                self.running = True
                self.start_threads()
                print("GPS-симулятор запущен")
                print("Дискретность: 100 мс, Вывод: 10 Гц")
                print("Для остановки нажмите Ctrl+C")
                return True
            else:
                print("Симулятор уже запущен")
                return False
    
    def stop(self):
        """
        Остановка симуляции.
        
        Устанавливает флаг остановки и ожидает завершения всех потоков.
        """
        with self.lock:
            if self.running:
                self.running = False
                print("Остановка симулятора...")
                
                # Ожидание завершения потоков
                if self.sim_thread and self.sim_thread.is_alive():
                    self.sim_thread.join(timeout=1.0)
                if self.output_thread and self.output_thread.is_alive():
                    self.output_thread.join(timeout=1.0)
                    
                print("Симулятор остановлен")
            else:
                print("Симулятор не запущен")
    
    def start_threads(self):
        """
        Создание и запуск рабочих потоков симулятора.
        
        Создает два потока для независимой работы
        симуляции движения и вывода NMEA сообщений.
        """
        # Поток симуляции (обновление позиции)
        self.sim_thread = threading.Thread(target=self.simulation_loop, daemon=True)
        self.sim_thread.start()
        
        # Поток вывода GPS (10 Гц)
        self.output_thread = threading.Thread(target=self.output_gps_messages, daemon=True)
        self.output_thread.start()
    
    def get_status(self):
        """
        Функция отображения текущего состояния 
        
        Выводит в консоль:
        - Текущие декартовы и GPS координаты транспорта
        - Курс движения
        - Скорость и параметры поворота
        - Расчетный радиус поворота
        """
        with self.lock:
            lat, lon = self.cartesian_to_gps(self.x, self.y)
            wheel_angle = self.calculate_wheel_angle(self.steering_angle)
            current_speed = self.speed_kmh
            current_steering = self.steering_angle
            current_heading = self.heading
            current_x, current_y = self.x, self.y
        
        print(f"\n=== Состояние симулятора ===")
        print(f"Декартовы координаты: ({current_x:.2f}, {current_y:.2f}) м")
        print(f"GPS координаты: {lat:.6f}, {lon:.6f}")
        print(f"Курс: {math.degrees(current_heading):.1f}°")
        print(f"Скорость: {current_speed} км/ч")
        print(f"Угол руля: {current_steering}° → угол колес: {wheel_angle:.1f}°")
        
        # Определение радиуса поворота
        if abs(current_steering) > 0.1:
            wheel_angle_rad = math.radians(wheel_angle)
            turn_radius = self.wheelbase / math.tan(abs(wheel_angle_rad))
            print(f"Радиус поворота: {turn_radius:.1f} м")
        else:
            print(f"Радиус поворота: нет – прямолинейное движение")
        print()


def main():
    """
    Главная функция программы.
    
    Создает экземпляр GPS-симулятора и обеспечивает интерактивное
    управление через командную строку.
    """
    print("GPS-имитатор согласно техническому заданию")
    print("=" * 60)
    print("Параметры:")
    print("- Угол руля: -600...+600 градусов") 
    print("- Скорость: 0-50 км/ч")
    print("- Wheelbase: 2 метра")
    print("- Дискретность: 100 мс")
    print("- Частота вывода: 10 Гц")
    print("- Начальные координаты: 53.262778, 50.372778")
    print("=" * 60)
    print()
    
    simulator = GPSSimulator()
    
    print("Команды:")
    print("  start             - запустить симулятор")
    print("  stop              - остановить симулятор")
    print("  speed <0-50>      - установить скорость (км/ч)")
    print("  angle <-600..600> - установить угол руля (градусы)")
    print("  reset             - сброс к исходным настройкам")
    print("  status            - показать состояние")
    print("  quit              - выход")
    print()
    print("Для экстренного выхода из симуляции нажмите Ctrl+C")
    print()
    
    try:
        while True:
            cmd = input("> ").strip().lower()
            
            if cmd == "quit":
                break
            elif cmd == "start":
                simulator.start()
            elif cmd == "stop":
                simulator.stop()
            elif cmd == "reset":
                simulator.reset()
            elif cmd == "status":
                simulator.get_status()
            elif cmd.startswith("speed "):
                try:
                    speed = cmd.split()[1]
                    simulator.set_speed(speed)
                except IndexError:
                    print("Формат: speed <число от 0 до 50>")
            elif cmd.startswith("angle "):
                try:
                    angle = cmd.split()[1]
                    simulator.set_steering_angle(angle)
                except IndexError:
                    print("Формат: angle <число от -600 до 600>")
            elif cmd:
                print("Неизвестная команда. Доступные: start, stop, speed, angle, reset, status, quit")
    
    except KeyboardInterrupt:
        print("\nЭкстренное прерывание пользователем")
    except Exception as e:
        print(f"Непредвиденная ошибка: {e}")
    finally:
        simulator.stop()
        print("Выход из программы")

if __name__ == "__main__":
    main()