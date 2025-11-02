import re
import json
import os
from datetime import datetime

def find_latest_asc_file(directory="."):
    """Find the most recent .asc file in the directory"""
    asc_files = [f for f in os.listdir(directory) if f.endswith('.asc')]
    if not asc_files:
        raise FileNotFoundError("No .asc files found in directory")
    
    # Get the most recent file based on modification time
    latest_file = max(asc_files, key=lambda f: os.path.getmtime(f))
    return latest_file

def parse_asc_file(file_path):
    """
    Parse ASC log file and extract CAN data with timestamps
    """
    config_params = {}
    time_series_data = {
        'timestamps': [],
        'packSOC': [],
        'packSOH': [],
        'packTemp': [],
        'packRange': [],
        'packCurrent': [],  # ADDED: Pack Current
        'acceleration': [],
        'cellSOC': [[], [], []],  # For 3 cells
        'cellSOH': [[], [], []]   # For 3 cells
    }
    
    # Regular expression for CAN messages
    can_message_pattern = r'^\s*(\d+\.\d+)\s+\d+\s+([0-9A-Fa-f]+)\s+(?:Rx|Tx)\s+[d]\s+(\d+)\s+((?:[0-9A-Fa-f]{2}\s*)+)'
    
    with open(file_path, 'r', encoding='utf-8', errors='ignore') as file:
        for line in file:
            line = line.strip()
            
            # Parse CAN messages
            can_match = re.match(can_message_pattern, line)
            if can_match:
                timestamp = float(can_match.group(1))
                can_id_hex = can_match.group(2)
                data_length = int(can_match.group(3))
                data_bytes = can_match.group(4).split()
                
                # Convert CAN ID from hex to decimal
                try:
                    can_id = int(can_id_hex, 16)
                except ValueError:
                    continue
                
                # Convert data bytes from hex to integers
                try:
                    data_values = [int(byte, 16) for byte in data_bytes[:data_length]]
                except ValueError:
                    continue
                
                # Process different CAN IDs based on your microcontroller code
                process_can_message(timestamp, can_id, data_values, time_series_data)
    
    return config_params, time_series_data

def process_can_message(timestamp, can_id, data, time_series):
    """
    Process CAN messages and extract relevant data based on CAN IDs
    """
    if not data:
        return
    
    # FIXED: Acceleration data - ONLY from CAN ID 0x20D (525 decimal)
    if can_id == 0x20D:  # Acceleration data
        if len(data) >= 1:
            # Add new timestamp and acceleration value
            time_series['timestamps'].append(timestamp)
            time_series['acceleration'].append(data[0])
            
            # For other parameters, add placeholder values to maintain array lengths
            # Use the last value if available, otherwise use default
            time_series['packSOC'].append(time_series['packSOC'][-1] if time_series['packSOC'] else 50)
            time_series['packSOH'].append(time_series['packSOH'][-1] if time_series['packSOH'] else 95)
            time_series['packTemp'].append(time_series['packTemp'][-1] if time_series['packTemp'] else 25)
            time_series['packRange'].append(time_series['packRange'][-1] if time_series['packRange'] else 200)
            time_series['packCurrent'].append(time_series['packCurrent'][-1] if time_series['packCurrent'] else 0)  # ADDED
            
            # Add placeholder cell data
            for i in range(3):
                time_series['cellSOC'][i].append(time_series['cellSOC'][i][-1] if time_series['cellSOC'][i] else 50)
                time_series['cellSOH'][i].append(time_series['cellSOH'][i][-1] if time_series['cellSOH'][i] else 95)
    
    # Battery status messages (0x201 to 0x20C)
    elif 0x201 <= can_id <= 0x20C:
        index = can_id - 0x201
        
        # If we have acceleration data but no battery data yet, create entries
        if time_series['timestamps'] and not time_series['packSOC']:
            # Initialize all arrays with placeholder values for existing timestamps
            for i in range(len(time_series['timestamps'])):
                time_series['packSOC'].append(50)
                time_series['packSOH'].append(95)
                time_series['packTemp'].append(25)
                time_series['packRange'].append(200)
                time_series['packCurrent'].append(0)  # ADDED
                for j in range(3):
                    time_series['cellSOC'][j].append(50)
                    time_series['cellSOH'][j].append(95)
        
        # Ensure we have timestamps array
        if not time_series['timestamps']:
            time_series['timestamps'].append(timestamp)
            # Initialize all arrays
            time_series['packSOC'].append(50)
            time_series['packSOH'].append(95)
            time_series['packTemp'].append(25)
            time_series['packRange'].append(200)
            time_series['packCurrent'].append(0)  # ADDED
            time_series['acceleration'].append(0)  # Default acceleration
            for i in range(3):
                time_series['cellSOC'][i].append(50)
                time_series['cellSOH'][i].append(95)
        
        # Extend arrays if needed
        current_length = len(time_series['timestamps'])
        while len(time_series['packSOC']) < current_length:
            time_series['packSOC'].append(time_series['packSOC'][-1] if time_series['packSOC'] else 50)
        while len(time_series['packSOH']) < current_length:
            time_series['packSOH'].append(time_series['packSOH'][-1] if time_series['packSOH'] else 95)
        while len(time_series['packTemp']) < current_length:
            time_series['packTemp'].append(time_series['packTemp'][-1] if time_series['packTemp'] else 25)
        while len(time_series['packRange']) < current_length:
            time_series['packRange'].append(time_series['packRange'][-1] if time_series['packRange'] else 200)
        while len(time_series['packCurrent']) < current_length:  # ADDED
            time_series['packCurrent'].append(time_series['packCurrent'][-1] if time_series['packCurrent'] else 0)
        while len(time_series['acceleration']) < current_length:
            time_series['acceleration'].append(time_series['acceleration'][-1] if time_series['acceleration'] else 0)
        
        # Process based on CAN ID mapping from your microcontroller code
        if index == 0:  # 0x201 - Pack SOC
            if len(data) >= 1:
                time_series['packSOC'][-1] = data[0]
                # Update all cell SOC values (assuming they're the same as pack SOC)
                for i in range(3):
                    if len(time_series['cellSOC'][i]) < current_length:
                        time_series['cellSOC'][i].append(data[0])
                    else:
                        time_series['cellSOC'][i][-1] = data[0]
        
        elif index == 1:  # 0x202 - Pack SOH
            if len(data) >= 1:
                time_series['packSOH'][-1] = data[0]
                # Update all cell SOH values
                for i in range(3):
                    if len(time_series['cellSOH'][i]) < current_length:
                        time_series['cellSOH'][i].append(data[0])
                    else:
                        time_series['cellSOH'][i][-1] = data[0]
        
        elif index == 2:  # 0x203 - Cell 0 SOC
            if len(data) >= 1:
                if len(time_series['cellSOC'][0]) < current_length:
                    time_series['cellSOC'][0].append(data[0])
                else:
                    time_series['cellSOC'][0][-1] = data[0]
        
        elif index == 3:  # 0x204 - Cell 1 SOC
            if len(data) >= 1:
                if len(time_series['cellSOC'][1]) < current_length:
                    time_series['cellSOC'][1].append(data[0])
                else:
                    time_series['cellSOC'][1][-1] = data[0]
        
        elif index == 4:  # 0x205 - Cell 2 SOC
            if len(data) >= 1:
                if len(time_series['cellSOC'][2]) < current_length:
                    time_series['cellSOC'][2].append(data[0])
                else:
                    time_series['cellSOC'][2][-1] = data[0]
        
        elif index == 5:  # 0x206 - Cell 0 SOH
            if len(data) >= 1:
                if len(time_series['cellSOH'][0]) < current_length:
                    time_series['cellSOH'][0].append(data[0])
                else:
                    time_series['cellSOH'][0][-1] = data[0]
        
        elif index == 6:  # 0x207 - Cell 1 SOH
            if len(data) >= 1:
                if len(time_series['cellSOH'][1]) < current_length:
                    time_series['cellSOH'][1].append(data[0])
                else:
                    time_series['cellSOH'][1][-1] = data[0]
        
        elif index == 7:  # 0x208 - Cell 2 SOH
            if len(data) >= 1:
                if len(time_series['cellSOH'][2]) < current_length:
                    time_series['cellSOH'][2].append(data[0])
                else:
                    time_series['cellSOH'][2][-1] = data[0]
        
        elif index == 8:  # 0x209 - Pack current
            if len(data) >= 1:  # CHANGED: Now processes current data
                time_series['packCurrent'][-1] = data[0]
        
        elif index == 9:  # 0x20A - Pack range
            if len(data) >= 1:
                time_series['packRange'][-1] = data[0]
        
        elif index == 10:  # 0x20B - Pack temperature (4 bytes)
            if len(data) >= 4:
                # Convert 4 bytes to integer (little endian)
                temp_value = data[0] | (data[1] << 8) | (data[2] << 16) | (data[3] << 24)
                # Handle signed integer
                if temp_value & 0x80000000:
                    temp_value = temp_value - (1 << 32)
                time_series['packTemp'][-1] = temp_value
        
        elif index == 11:  # 0x20C - Fault flags (4 bytes)
            # Not directly used in current charts
            pass

def create_default_config():
    """Create default configuration based on your microcontroller defaults"""
    return {
        'batteryCapacity': 100,
        'numCells': 3,
        'cellCapacity': [33, 33, 33],
        'cellPower': [200, 200, 200],
        'initialSOC': [100, 100, 100],
        'initialSOH': [100, 100, 100],
        'degradationRate': [1, 1, 1],
        'maxDischargeCurrent': 120,
        'maxChargeCurrent': 100
    }

def ensure_data_consistency(time_series):
    """Ensure all arrays have the same length"""
    max_length = len(time_series['timestamps'])
    
    # Ensure all main arrays have the same length
    for key in ['packSOC', 'packSOH', 'packTemp', 'packRange', 'packCurrent', 'acceleration']:  # ADDED packCurrent
        while len(time_series[key]) < max_length:
            if time_series[key]:
                time_series[key].append(time_series[key][-1])
            else:
                # Default values if array is empty
                if key == 'packSOC':
                    time_series[key].append(50)
                elif key == 'packSOH':
                    time_series[key].append(95)
                elif key == 'packTemp':
                    time_series[key].append(25)
                elif key == 'packRange':
                    time_series[key].append(200)
                elif key == 'packCurrent':  # ADDED
                    time_series[key].append(0)
                else:  # acceleration
                    time_series[key].append(0)
    
    # Ensure cell arrays have the same length
    for i in range(3):
        while len(time_series['cellSOC'][i]) < max_length:
            if time_series['cellSOC'][i]:
                time_series['cellSOC'][i].append(time_series['cellSOC'][i][-1])
            else:
                time_series['cellSOC'][i].append(time_series['packSOC'][-1] if time_series['packSOC'] else 50)
        
        while len(time_series['cellSOH'][i]) < max_length:
            if time_series['cellSOH'][i]:
                time_series['cellSOH'][i].append(time_series['cellSOH'][i][-1])
            else:
                time_series['cellSOH'][i].append(time_series['packSOH'][-1] if time_series['packSOH'] else 95)

def main():
    try:
        # Find the latest .asc file automatically
        asc_file = find_latest_asc_file()
        print(f"Processing file: {asc_file}")
        
        # Parse the ASC file
        config_params, time_series_data = parse_asc_file(asc_file)
        
        print(f"CAN messages processed")
        print(f"Time series data points: {len(time_series_data['timestamps'])}")
        print(f"Pack SOC data points: {len(time_series_data['packSOC'])}")
        print(f"Pack SOH data points: {len(time_series_data['packSOH'])}")
        print(f"Temperature data points: {len(time_series_data['packTemp'])}")
        print(f"Pack Current data points: {len(time_series_data['packCurrent'])}")  # ADDED
        print(f"Acceleration data points: {len(time_series_data['acceleration'])}")
        
        # Print acceleration data sample for verification
        if time_series_data['acceleration']:
            print(f"Acceleration data range: {min(time_series_data['acceleration'])} to {max(time_series_data['acceleration'])}")
            print(f"First 20 acceleration values: {time_series_data['acceleration'][:20]}")
        
        # Print pack current data sample for verification
        if time_series_data['packCurrent']:  # ADDED
            print(f"Pack Current range: {min(time_series_data['packCurrent'])} to {max(time_series_data['packCurrent'])} A")
            print(f"First 20 current values: {time_series_data['packCurrent'][:20]}")
        
        # Ensure data consistency
        ensure_data_consistency(time_series_data)
        
        # Create default configuration
        config_data = create_default_config()
        
        # Prepare final data structure
        processed_data = {
            'config': config_data,
            'time_series': time_series_data
        }
        
        # Get the absolute path to the website data directory
        script_dir = os.path.dirname(os.path.abspath(__file__))
        website_data_dir = os.path.join(script_dir, '..', 'website', 'data')
        os.makedirs(website_data_dir, exist_ok=True)

        # Save to JSON
        output_json = os.path.join(website_data_dir, 'processed_data.json')
        with open(output_json, 'w') as f:
            json.dump(processed_data, f, indent=2)

        print(f"Processed data saved to: {output_json}")
        print("Data processing completed!")
        
    except FileNotFoundError as e:
        print(f"Error: {e}")
    except Exception as e:
        print(f"Error processing file: {e}")

if __name__ == "__main__":
    main()
