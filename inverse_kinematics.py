from math import acos, atan2, cos, sin, radians, pi

RAD_TO_DEG = 57.2957786

FRAME_TIME_MS = 20  # время кадра (20мсек = 50Hz)

COXA_LENGTH = 83.5  # длин частей ног
FEMUR_LENGTH = 90
TIBIA_LENGTH = 154  # 157

TRAVEL = 30  # предельная константа перемещение и вращение

A12DEG = 209440  # 12 градусов в радианах x 1.000.000
A30DEG = 523599  # 30 градусов в радианах x 1.000.000

# coxa-to-toe исходная позиция 122.683,   0.0, -122.683,  -122.683,    0.0,  122.683
HOME_X = (141.4, 0.0, -141.4, -141.4, 0.0, 141.4)
HOME_Y = (141.4, 200, 141.4, -141.4, -200, -141.4)  # 122.683, 173.5,  122.683,  -122.683, -173.5, -122.683
HOME_Z = (-120.0, -120.0, -120.0, -120.0, -120.0, -120.0)

BODY_X = (192.75, 0.0, -192.75, -192.75, 0.0, 192.75)  # Расстояния сервопривода от центра тела до coxa
BODY_Y = (80.5, 80.5, 80.5, -80.5, -80.5, -80.5)
BODY_Z = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)

COXA_CAL = (2, -1, -1, -3, -2, -3)  # константы калибровки сервопривода
FEMUR_CAL = (4, -2, 0, -1, 0, 0)
TIBIA_CAL = (0, -3, -3, -2, -3, -1)

tripod_case = [1, 2, 1, 2, 1, 2]  
ripple_case = [2, 6, 4, 1, 3, 5]  
wave_case = [1, 2, 3, 4, 5, 6]  
tetrapod_case = [1, 3, 2, 1, 2, 3]  


def constrain(val, min_val, max_val):
    return min(max_val, max(min_val, val))


def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


class InverseKinematics:

    def __init__(self):
        self.tick = 0
        self.duration = 0
        self.currentX = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.currentY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.currentZ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.offsetX = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.offsetY = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.offsetZ = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.step_height_multiplier = 0
        self.gaitSpeed = 0
        self.servo = [[0, 0, 0],  # 18 углы поворота 18 сервоприводов
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0],
                      [0, 0, 0]]

    @staticmethod
    def leg_ik(leg_number, x, y, z):
        # вычисление femur-to-toe (l3) длинны
        l0 = (x ** 2 + y ** 2) ** 0.5 - COXA_LENGTH  # переменные инверсной кинематики
        l3 = (l0 ** 2 + z ** 2) ** 0.5
        # обрабатывайте только в том случае, если радиус действия находится в пределах возможного диапазона
        if (l3 < (TIBIA_LENGTH + FEMUR_LENGTH)) and (l3 > (TIBIA_LENGTH - FEMUR_LENGTH)):
            # Вычисление угла tibia
            phi_tibia = acos((FEMUR_LENGTH ** 2 + TIBIA_LENGTH ** 2 - l3 ** 2) / (2 * FEMUR_LENGTH * TIBIA_LENGTH))
            theta_tibia = phi_tibia * RAD_TO_DEG - 23.0 + TIBIA_CAL[leg_number]
            theta_tibia = constrain(theta_tibia, 0.0, 180.0)
            # Вычисление угла femur
            gamma_femur = atan2(z, l0)
            phi_femur = acos((FEMUR_LENGTH ** 2 + l3 ** 2 - TIBIA_LENGTH ** 2) / (2 * FEMUR_LENGTH * l3))
            theta_femur = (phi_femur + gamma_femur) * RAD_TO_DEG + 14.0 + 90.0 + FEMUR_CAL[leg_number]
            theta_femur = constrain(theta_femur, 0.0, 180.0)
            # Вычисление угла coxa
            theta_coxa = atan2(x, y) * RAD_TO_DEG + COXA_CAL[leg_number]
            # вывод на соответствующую ногу
            if leg_number == 0:
                theta_coxa = theta_coxa + 45.0  # компенсация крепления ноги
                theta_coxa = constrain(theta_coxa, 0.0, 180.0)
                return int(theta_coxa), int(180 - theta_femur), int(theta_tibia)
            elif leg_number == 1:
                theta_coxa = theta_coxa + 90.0  # компенсация крепления ноги
                theta_coxa = constrain(theta_coxa, 0.0, 180.0)
                return int(theta_coxa), int(180 - theta_femur), int(theta_tibia)
            elif leg_number == 2:
                theta_coxa = theta_coxa + 135.0  # компенсация крепления ноги
                theta_coxa = constrain(theta_coxa, 0.0, 180.0)
                return int(theta_coxa), int(180 - theta_femur), int(theta_tibia)
            elif leg_number == 3:
                if theta_coxa < 0:                    # компенсация крепления ноги
                    theta_coxa = theta_coxa + 225.0   # (необходимо использовать разные
                else:                                 # положительные и отрицательные смещения из-за
                    theta_coxa = theta_coxa - 135.0   # приведенных выше результатов atan2!)
                theta_coxa = constrain(theta_coxa, 0.0, 180.0)
                return int(theta_coxa), int(theta_femur), int(180 - theta_tibia)
            elif leg_number == 4:
                if theta_coxa < 0:                     # компенсация крепления ноги
                    theta_coxa = theta_coxa + 270.0    # (необходимо использовать разные
                else:                                  # положительные и отрицательные смещения из-за
                    theta_coxa = theta_coxa - 90.0     # приведенных выше результатов atan2!)
                theta_coxa = constrain(theta_coxa, 0.0, 180.0)
                return int(theta_coxa), int(theta_femur), int(180 - theta_tibia)
            elif leg_number == 5:
                if theta_coxa < 0:                      # компенсация крепления ноги
                    theta_coxa = theta_coxa + 315.0     # (необходимо использовать разные
                else:                                   # положительные и отрицательные смещения из-за
                    theta_coxa = theta_coxa - 45.0      # приведенных выше результатов atan2!)
                theta_coxa = constrain(theta_coxa, 0.0, 180.0)
                return int(theta_coxa), int(theta_femur), int(180 - theta_tibia)

    def tripod_gait(self, sidestep, rotation, direction):
        commanded_x = arduino_map(direction, 0, 255, 127, -127)
        commanded_y = arduino_map(sidestep, 0, 255, -127, 127)
        commanded_r = arduino_map(rotation, 0, 255, 127, -127)
        # если команды превышают зону нечувствительности, тогда обрабатывайте
        if (abs(commanded_x) > 15) or (abs(commanded_y) > 15) or (abs(commanded_r) > 15) or (self.tick > 0):
            step_x, step_y, step_r = self.compute_strides(commanded_x, commanded_y, commanded_r)
            num_ticks = round(self.duration / FRAME_TIME_MS / 2)  # общее количество тиков, разделенное на два случая
            for leg_num in range(6):
                self.step_height_multiplier = 2 # Регулировка высоты шага (стандартное значение: 1,0, максимальное: 2,0)
                amplitude_x, amplitude_y, amplitude_z = self.compute_amplitudes(step_x, step_y, step_r, leg_num)
                if tripod_case[leg_num] == 1:  # move foot forward (raise and lower)
                    self.currentX[leg_num] = HOME_X[leg_num] - amplitude_x * cos(pi * self.tick / num_ticks)
                    self.currentY[leg_num] = HOME_Y[leg_num] - amplitude_y * cos(pi * self.tick / num_ticks)
                    self.currentZ[leg_num] = HOME_Z[leg_num] + abs(amplitude_z) * sin(pi * self.tick / num_ticks)
                    if self.tick >= num_ticks - 1:
                        tripod_case[leg_num] = 2
                elif tripod_case[leg_num] == 2:  # move foot back (on the ground)
                    self.currentX[leg_num] = HOME_X[leg_num] + amplitude_x * cos(pi * self.tick / num_ticks)
                    self.currentY[leg_num] = HOME_Y[leg_num] + amplitude_y * cos(pi * self.tick / num_ticks)
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        tripod_case[leg_num] = 1

            # увеличить тик
            if self.tick < num_ticks - 1:
                self.tick += 1
            else:
                self.tick = 0

    def wave_gait(self, sidestep, rotation, direction):
        commanded_x = arduino_map(direction, 0, 255, 127, -127)
        commanded_y = arduino_map(sidestep, 0, 255, -127, 127)
        commanded_r = arduino_map(rotation, 0, 255, 127, -127)
        # если команды превышают зону нечувствительности, тогда обрабатываем
        if (abs(commanded_x) > 15) or (abs(commanded_y) > 15) or (abs(commanded_r) > 15) or (self.tick > 0):
            step_x, step_y, step_r = self.compute_strides(commanded_x, commanded_y, commanded_r)
            # общее количество тиков, разделенное на шесть случаев
            num_ticks = round(self.duration / FRAME_TIME_MS / 6.0)
            for leg_num in range(6):
                self.step_height_multiplier = 2.0  # регулировка высоты шага
                amplitude_x, amplitude_y, amplitude_z = self.compute_amplitudes(step_x, step_y, step_r, leg_num)
                if wave_case[leg_num] == 1:  # move foot forward (raise and lower)
                    self.currentX[leg_num] = HOME_X[leg_num] - amplitude_x * cos(pi * self.tick / num_ticks)
                    self.currentY[leg_num] = HOME_Y[leg_num] - amplitude_y * cos(pi * self.tick / num_ticks)
                    self.currentZ[leg_num] = HOME_Z[leg_num] + abs(amplitude_z) * sin(pi * self.tick / num_ticks)
                    if self.tick >= num_ticks - 1:
                        wave_case[leg_num] = 6
                elif wave_case[leg_num] == 2:  # move foot back one-fifth (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2.5
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2.5
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        wave_case[leg_num] = 1
                elif wave_case[leg_num] == 3:  # move foot back one-fifth (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2.5
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2.5
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        wave_case[leg_num] = 2
                elif wave_case[leg_num] == 4:  # move foot back one-fifth (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2.5
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2.5
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        wave_case[leg_num] = 3
                elif wave_case[leg_num] == 5:  # move foot back one-fifth (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2.5
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2.5
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        wave_case[leg_num] = 4
                elif wave_case[leg_num] == 6:  # move foot back one-fifth (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2.5
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2.5
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        wave_case[leg_num] = 5
            # increment tick
            if self.tick < num_ticks - 1:
                self.tick += 1
            else:
                self.tick = 0

    def ripple_gait(self, sidestep, rotation, direction):
        commanded_x = arduino_map(direction, 0, 255, 127, -127)
        commanded_y = arduino_map(sidestep, 0, 255, -127, 127)
        commanded_r = arduino_map(rotation, 0, 255, 127, -127)

        # если команды превышают зону нечувствительности, тогда обрабатывайте
        if (abs(commanded_x) > 15) or (abs(commanded_y) > 15) or (abs(commanded_r) > 15) or (self.tick > 0):
            step_x, step_y, step_r = self.compute_strides(commanded_x, commanded_y, commanded_r)
            num_ticks = round(self.duration / FRAME_TIME_MS / 6.0)  # total ticks divided into the six cases
            for leg_num in range(6):
                self.step_height_multiplier = 3.0  # Adjusting the step height (Standard value: 1.0, maximum: 2.0)
                amplitude_x, amplitude_y, amplitude_z = self.compute_amplitudes(step_x, step_y, step_r, leg_num)
                if ripple_case[leg_num] == 1:  # move foot forward (raise)
                    self.currentX[leg_num] = HOME_X[leg_num] - amplitude_x * cos(pi * self.tick / (num_ticks * 2))
                    self.currentY[leg_num] = HOME_Y[leg_num] - amplitude_y * cos(pi * self.tick / (num_ticks * 2))
                    self.currentZ[leg_num] = HOME_Z[leg_num] + abs(amplitude_z) * sin(pi * self.tick / (num_ticks * 2))
                    if self.tick >= num_ticks - 1:
                        ripple_case[leg_num] = 2
                elif ripple_case[leg_num] == 2:  # move foot forward (lower)
                    self.currentX[leg_num] = HOME_X[leg_num] - amplitude_x * cos(pi * (num_ticks + self.tick) / (num_ticks * 2))
                    self.currentY[leg_num] = HOME_Y[leg_num] - amplitude_y * cos(pi * (num_ticks + self.tick) / (num_ticks * 2))
                    self.currentZ[leg_num] = HOME_Z[leg_num] + abs(amplitude_z) * sin(pi * (num_ticks + self.tick) / (num_ticks * 2))
                    if self.tick >= num_ticks - 1:
                        ripple_case[leg_num] = 3
                elif ripple_case[leg_num] == 3:  # move foot back one-quarter (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        ripple_case[leg_num] = 4
                elif ripple_case[leg_num] == 4:  # move foot back one-quarter (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        ripple_case[leg_num] = 5
                elif ripple_case[leg_num] == 5:  # move foot back one-quarter (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        ripple_case[leg_num] = 6
                elif ripple_case[leg_num] == 6:  # move foot back one-quarter (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - amplitude_x / num_ticks / 2
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks / 2
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        ripple_case[leg_num] = 1

            # инкриминируем тик
            if self.tick < num_ticks - 1:
                self.tick += 1
            else:
                self.tick = 0

    def tetrapod_gait(self, sidestep, rotation, direction):
        commanded_x = arduino_map(direction, 0, 255, 127, -127)
        commanded_y = arduino_map(sidestep, 0, 255, -127, 127)
        commanded_r = arduino_map(rotation, 0, 255, 127, -127)
        # если команды превышают зону нечувствительности, тогда обрабатывайте
        if (abs(commanded_x) > 15) or (abs(commanded_y) > 15) or (abs(commanded_r) > 15) or (self.tick > 0):
            step_x, step_y, step_r = self.compute_strides(commanded_x, commanded_y, commanded_r)
            num_ticks = round(self.duration / FRAME_TIME_MS / 3.0)  # total ticks divided into the three cases
            for leg_num in range(6):
                self.step_height_multiplier = 2.5  # Adjusting the step height (Standard value: 1.0, maximum: 2.0)
                samplitude_x, amplitude_y, amplitude_z = self.compute_amplitudes(step_x, step_y, step_r, leg_num)
                if tetrapod_case[leg_num] == 1:  # move foot forward (raise and lower)
                    self.currentX[leg_num] = HOME_X[leg_num] - samplitude_x * cos(pi * self.tick / num_ticks)
                    self.currentY[leg_num] = HOME_Y[leg_num] - amplitude_y * cos(pi * self.tick / num_ticks)
                    self.currentZ[leg_num] = HOME_Z[leg_num] + abs(amplitude_z) * sin(pi * self.tick / num_ticks)
                    if self.tick >= num_ticks - 1:
                        tetrapod_case[leg_num] = 2
                elif tetrapod_case[leg_num] == 2:  # move foot back one-half (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - samplitude_x / num_ticks
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        tetrapod_case[leg_num] = 3
                elif tetrapod_case[leg_num] == 3:  # move foot back one-half (on the ground)
                    self.currentX[leg_num] = self.currentX[leg_num] - samplitude_x / num_ticks
                    self.currentY[leg_num] = self.currentY[leg_num] - amplitude_y / num_ticks
                    self.currentZ[leg_num] = HOME_Z[leg_num]
                    if self.tick >= num_ticks - 1:
                        tetrapod_case[leg_num] = 1

            # Прибавляем тик
            if self.tick < num_ticks - 1:
                self.tick += 1
            else:
                self.tick = 0

    def rotate_control(self, x, y, z, capture_offsets):
        # вычисление смещения по X
        translate_x = arduino_map(x, 0, 255, -2 * TRAVEL, 2 * TRAVEL)
        for legNum in range(6):
            self.currentX[legNum] = HOME_X[legNum] + translate_x
        # вычисление смещения по Y
        translate_y = arduino_map(y, 0, 255, 2 * TRAVEL, -2 * TRAVEL)
        for legNum in range(6):
            self.currentY[legNum] = HOME_Y[legNum] + translate_y
        # вычисление смещения по Z
        translate_z = z
        if translate_z > 127:
            translate_z = arduino_map(translate_z, 128, 255, 0, TRAVEL)
        else:
            translate_z = arduino_map(translate_z, 0, 127, -3 * TRAVEL, 0)

        for legNum in range(6):
            self.currentZ[legNum] = HOME_Z[legNum] + translate_z

        # lock in offsets if commanded
        if capture_offsets:
            for legNum in range(6):
                self.offsetX[legNum] = self.offsetX[legNum] + translate_x
                self.offsetY[legNum] = self.offsetY[legNum] + translate_y
                self.offsetZ[legNum] = self.offsetZ[legNum] + translate_z
                self.currentX[legNum] = HOME_X[legNum]
                self.currentY[legNum] = HOME_Y[legNum]
                self.currentZ[legNum] = HOME_Z[legNum]

    def translate_control(self, x, y, z, capture_offsets):
        # вычисление смещения по X
        translate_x = arduino_map(x, 0, 255, -2 * TRAVEL, 2 * TRAVEL)
        for legNum in range(6):
            self.currentX[legNum] = HOME_X[legNum] + translate_x
        # вычисление смещения по Y
        translate_y = arduino_map(y, 0, 255, 2 * TRAVEL, -2 * TRAVEL)
        for legNum in range(6):
            self.currentY[legNum] = HOME_Y[legNum] + translate_y
        # вычисление смещения по Z
        translate_z = z
        if translate_z > 127:
            translate_z = arduino_map(translate_z, 128, 255, 0, TRAVEL)
        else:
            translate_z = arduino_map(translate_z, 0, 127, -3 * TRAVEL, 0)
        for legNum in range(6):
            self.currentZ[legNum] = HOME_Z[legNum] + translate_z

        # фиксировать смещение по команде
        if capture_offsets:
            for legNum in range(6):
                self.offsetX[legNum] = self.offsetX[legNum] + translate_x
                self.offsetY[legNum] = self.offsetY[legNum] + translate_y
                self.offsetZ[legNum] = self.offsetZ[legNum] + translate_z
                self.currentX[legNum] = HOME_X[legNum]
                self.currentY[legNum] = HOME_Y[legNum]
                self.currentZ[legNum] = HOME_Z[legNum]

    def set_all_90(self):
        for leg_num in range(6):
            self.servo[leg_num] = [90 + COXA_CAL[leg_num], 90 + FEMUR_CAL[leg_num], 90 + TIBIA_CAL[leg_num]]

    def reset_position(self):
        for leg_num in range(6):
            self.currentX[leg_num] = HOME_X[leg_num]
            self.currentY[leg_num] = HOME_Y[leg_num]
            self.currentZ[leg_num] = HOME_Z[leg_num]

    def update(self):
        for leg_num in range(6):
            self.servo[leg_num] = list(self.leg_ik(leg_num,
                                                   self.currentX[leg_num] + self.offsetX[leg_num],
                                                   self.currentY[leg_num] + self.offsetY[leg_num],
                                                   self.currentZ[leg_num] + self.offsetZ[leg_num]))

    def compute_strides(self, x, y, r):
        # установить продолжительность для нормального и медленного режимов скорости
        if self.gaitSpeed == 0:
            self.duration = 720  # 1080, 960, 840, 720, 600, 480
        else:
            self.duration = 3240

        # вычислить длину шага
        return 90 * x / 127, 90 * y / 127, 35 * r / 127

    def compute_amplitudes(self, stride_x, stride_y, stride_r, leg_num):
        # вычислить триггер вращения
        sin_rot_z = sin(radians(stride_r))
        cos_rot_z = cos(radians(stride_r))

        # вычислить общее расстояние от центра тела до пальцев ног
        total_x = HOME_X[leg_num] + BODY_X[leg_num]
        total_y = HOME_Y[leg_num] + BODY_Y[leg_num]

        # вычислить смещение вращения
        rot_offset_x = total_y * sin_rot_z + total_x * cos_rot_z - total_x
        rot_offset_y = total_y * cos_rot_z - total_x * sin_rot_z - total_y

        # вычислить амплитуду X и Y и ограничить, чтобы ноги не врезались друг в друга
        amplitude_x = (stride_x + rot_offset_x) / 2
        amplitude_y = (stride_y + rot_offset_y) / 2
        amplitude_x = constrain(amplitude_x, -50, 50)
        amplitude_y = constrain(amplitude_y, -50, 50)

        # вычислить амплитуду по Z
        if abs(stride_x + rot_offset_x) > abs(stride_y + rot_offset_y):
            amplitude_z = self.step_height_multiplier * (stride_x + rot_offset_x) / 4
        else:
            amplitude_z = self.step_height_multiplier * (stride_y + rot_offset_y) / 4

        return amplitude_x, amplitude_y, amplitude_z
