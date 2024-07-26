import numpy as np
import csv


def set_params(name):
    # Define the headers and rows
    headers = ['Parmater', 'Value']
    rows = [
        ['T_c', 20],
        ['T_k', 293.15],
        ['P_atm', 101325],
        ['mu', 1.81e-5],
        ['v', 1.48e-5],
        ['rho', 1.225],
        ['S/2', 10],
        ['b/2', 15],
        ['c', 1.5],
        ['Re_t', 1e6],
        ['w', 9810],
        ['m', 1000]
    ]

    # Define the file path
    csv_file_path = name + '.csv'

    # Write to the CSV file
    with open(csv_file_path, 'w', newline='') as csvfile:
        csvwriter = csv.writer(csvfile)
        csvwriter.writerow(headers)
        csvwriter.writerows(rows)

    print(f"CSV file saved at: {csv_file_path}")


def generate_prefix():
    print('TESTS:')

    print('Gimbal Test          (1)\n')

    print('Control Surface Test (2)\n')

    test_labels = {'1': 'Gimbal', '2': 'ControlSurface'}

    test_info = int(input('Choose Test: '))

    if test_info == 1:

        dof_info = input('Input DOF {Roll, Pitch, YAW}: ')  # i.e. "101"
        
        prefix = test_labels[str(test_info)] + '_' + str(dof_info)


    if test_info == 2:

        controlsurface_info = input('Input Tested Control Surface {Elevator (E), Aileron (A), Rudder (R)}]: ') # i.e. "E"

        prefix =  test_labels[str(test_info)] + '_' + str(controlsurface_info)

    return prefix


def test_info2path(log_filename, parameter_filename):

    timestamp = (log_filename.split('.csv')[0]).split('_')[-1]

    type, detail = parameter_filename.split('_')[0], parameter_filename.split('_')[1]

    if type == 'Gimbal':
        no_dof = sum(int(digit) for digit in detail)

        folder_path = f"{no_dof}DOFGimbal"
    
    if type == 'ControlSurface':

        folder_path = f"Info_delta_{detail}"

        sub_folder_path = folder_path + "/" + timestamp

    return sub_folder_path, folder_path
