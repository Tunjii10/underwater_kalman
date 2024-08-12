#!/usr/bin/env python3

import pandas as pd
from datetime import datetime

# Define a function to parse and sort the data
def process_csv(file_path):
    # Load the CSV file into a pandas DataFrame
    df = pd.read_csv(file_path)

    # Create empty lists to store the split rows
    imu_rows = []
    dvl_rows = []
    filter_rows = []

    # Iterate through each row in the original DataFrame
    for _, row in df.iterrows():
        # Split the row into three parts
        imu_row = {
            'timestamp': row['t_imu'], 
            'type': 'IMU', 
            'AccX': row['AccX'], 'AccY': row['AccY'], 'AccZ': row['AccZ'], 
            'GyrX': row['GyrX'], 'GyrY': row['GyrY'], 'GyrZ': row['GyrZ'], 
            'MagX': row['MagX'], 'MagY': row['MagZ'], 'MagZ': row['MagZ'], 
            'pressure': row['pressure'], 'temperature': row['temperature']
        }
        imu_rows.append(imu_row)

        dvl_row = {
            'timestamp': row['t_DVL'], 
            'type': 'DVL', 
            'WX': row['WX'], 'WY': row['WY'], 'WZ': row['WZ'], 
            'BX': row['BX'], 'BY': row['BY'], 'BZ': row['BZ']
        }
        dvl_rows.append(dvl_row)

        filter_row = {
            'timestamp': row['t_filter'], 
            'type': 'Filter', 
            'PN': row['PN'], 'PE': row['PE'], 'PD': row['PD']
        }
        filter_rows.append(filter_row)

    # Combine the lists into a new DataFrame
    combined_df = pd.DataFrame(imu_rows + dvl_rows + filter_rows)

    # Convert the 'timestamp' column to datetime format
    combined_df['timestamp'] = pd.to_datetime(combined_df['timestamp'], format='%Y/%m/%d/%H:%M:%S.%f')

    # Drop any rows without a timestamp value
    combined_df = combined_df.dropna(subset=['timestamp'])

    # Sort the DataFrame by the timestamp
    combined_df = combined_df.sort_values(by='timestamp').reset_index(drop=True)

    # Optionally, write the processed data to a new CSV
    combined_df.to_csv('processed_data.csv', index=False)

    return combined_df

# Example usage
processed_df = process_csv('../data/AUV-Alice/AliceData2.csv')
print(processed_df.head())  # Display the first few rows of the processed data
