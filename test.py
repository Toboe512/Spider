from inverse_kinematics import InverseKinematics

if __name__ == '__main__':
    ik = InverseKinematics()
    ik.update()
    ik.wave_gait(127, 0, 0)
    result = ''
    for s in ik.servo:
        for i in s:
            result += str(i) + ' '
    print(f'{result} \n')

