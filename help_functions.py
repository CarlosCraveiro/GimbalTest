import numpy as np
import os
import csv
import math

# Gravitational acceleration
def gravitational_acceleration(latitude, altitude):
    g_0 = 9.80665  # Standard gravitational acceleration (m/s²)
    beta = 0.0053024  # Factor accounting for the variation with latitude
    gamma = 3.086e-6  # Factor accounting for the variation with altitude (per meter)
    
    latitude_rad = math.radians(latitude)
    g = g_0 * (1 + beta * math.sin(latitude_rad)**2 - gamma * altitude)
    return g


# Constants Environmental
kelvin_at_0_celcius = 273.15    # Kelvin
latitude = -22.01  # Latitude of São Carlos, Brazil
altitude = 856  # Altitude of São Carlos, Brazil in meters
g = gravitational_acceleration(latitude, altitude)


# System parameters
bhalf = 798.5 * 10**-3          # m
c_mean = 256 * 10**-3           # m
Shalf = bhalf * c_mean          # m^2


# WORKSPACE=================================================================================
# ==========================================================================================


# Meassured Units (Slow change)
T_c = 22.5                      # Celsius                                               EVERY TIME
Humidity = 0.45                 # Relative humidity (example value, update as needed)   EVERY 10 Min (5:45)
P_s_mbar = 928                  # mbar                                                  EVERY 10 Min (5:45)

# Meassured Units (Rapid change)
F_prop = 50                     # Frequency in Hz (example value, update as needed)
P_dyn =  0                      # Dynamic pressure in Pa (example value, update as needed)


# WORKSPACE=================================================================================
# ==========================================================================================

# Viscosity calculations
def saturation_vapor_pressure(T):
    T_C = T - 273.15  # Convert Kelvin to Celsius
    return 6.1078e3 * math.exp((17.27 * T_C) / (T_C + 35.85))

def calculate_viscosities(T, p, phi):
    mu_0 = 1.81e-5  # Reference dynamic viscosity at T0 (Pa·s)
    T0 = 273.15  # Reference temperature (K)
    S = 110.4  # Sutherland's constant for air (K)
    R_d = 287.05  # Specific gas constant for dry air (J/(kg·K))
    R_v = 461.495  # Specific gas constant for water vapor (J/(kg·K))

    p_sat = saturation_vapor_pressure(T)
    p_v = phi * p_sat
    p_d = p - p_v
    rho = (p_d / (R_d * T)) + (p_v / (R_v * T))
    mu = mu_0 * ((T / T0) ** 1.5) * (T0 + S) / (T + S)
    nu = mu / rho

    return mu, nu, rho

T_k = T_c + kelvin_at_0_celcius
P_atm = P_s_mbar * 100  # Convert mbar to Pa
phi = Humidity  # Update with actual humidity value

mu, nu, density = calculate_viscosities(T_k, P_atm, phi)

# Define other parameters
Re_target = 544629.3495  # Target Reynolds number
m = 3.0  # Example mass in kg
weight = m * g

# Prepare data for CSV
def set_params(file_path):
    headers = ['Parameter', 'Value']
    rows = [
        ['T_c', T_c],               
        ['T_k', T_k],
        ['P_atm', P_atm],
        ['mu', mu],
        ['nu', nu],
        ['rho', density],
        ['S/2', Shalf],
        ['b/2', bhalf],
        ['c', c_mean],
        ['Re_t', Re_target],
        ['m', m],
        ['w', weight],
        ['F_prop', F_prop],
        ['Humidity', Humidity],
        ['P_dyn', P_dyn]
    ]
    
    # Extract directory from file path
    directory = os.path.dirname(file_path)
    
    # Create the directory if it does not exist
    if not os.path.exists(directory):
        os.makedirs(directory)
    
    # Write the CSV file
    with open(file_path, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(headers)
        writer.writerows(rows)

    print(f"Parameter CSV file saved at: {file_path}")

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

def test_info2path(log_filename):

    timestamp = (log_filename.split('.csv')[0]).split('_')[-1]

    type, detail = log_filename.split('_')[0], log_filename.split('_')[1]

    if type == 'Gimbal':
        no_dof = sum(int(digit) for digit in detail)

        folder_path = f"{no_dof}DOFGimbal"
    
    if type == 'ControlSurface':

        folder_path = f"Info_delta_{detail}"

    
    folder_path = folder_path + "/" + timestamp


    print(f"Logfile CSV file saved at:", folder_path + '/' + log_filename)

    return folder_path
