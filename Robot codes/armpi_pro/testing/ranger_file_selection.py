import os
import subprocess

def ranger_file_selection():
    temp_file_for_ranger_path = "/tmp/chosen_file.txt"
    try:
        # Ensure the temp file doesn't exist from a previous run
        if os.path.exists(temp_file_for_ranger_path):
            os.remove(temp_file_for_ranger_path)

        print("[INFO] Launching Ranger for file selection.")
        
        # Run Ranger, telling it to write the chosen file's path to our temp file
        process = subprocess.Popen(['ranger', '--choosefile=' + temp_file_for_ranger_path])
        process.wait()  # Wait for the user to close Ranger

        # After Ranger closes, check if a file was written to the temp file
        if os.path.exists(temp_file_for_ranger_path):
            with open(temp_file_for_ranger_path, 'r') as f:
                selected_file = f.read().strip()
                print(f"[INFO] File selected: {selected_file}")
                return selected_file
        else:
            print("[INFO] Ranger was closed without selecting a file.")
            return None
            
    except FileNotFoundError:
        print("[ERROR] The 'ranger' command was not found.")
        return None
    except Exception as e:
        print(f"[ERROR] An error occurred while running Ranger: {e}")
        return None
    finally:
        # Clean up the temporary file
        if os.path.exists(temp_file_for_ranger_path):
            os.remove(temp_file_for_ranger_path)
