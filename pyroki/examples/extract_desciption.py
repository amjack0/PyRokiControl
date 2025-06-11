import os
import numpy as np

def get_description_files():
    """
    Detects files in the current directory whose names end with '_description.py',
    and returns their names (without the .py extension) in a list and a NumPy array.
    """
    description_filenames = []
    
    # List all files and directories in the current directory
    for filename in os.listdir('.'):
        # Check if it's a file and its name ends with '_description.py'
        if os.path.isfile(filename) and filename.endswith('_description.py'):
            # Extract the filename without the '.py' extension
            name_without_extension = filename[:-3] 
            description_filenames.append(name_without_extension)
            
    # Convert the list to a NumPy array
    description_filenames_np = np.array(description_filenames)
    
    return description_filenames, description_filenames_np

if __name__ == "__main__":
    file_list, file_numpy_array = get_description_files()
    
    print("Files ending with '_description.py' (as a list):")
    print(file_list)
    print("\nFiles ending with '_description.py' (as a NumPy array):")
    print(file_numpy_array)

    # Example of how to use it with the 'panda_description'
    # if 'panda_description' in file_list:
    #    print("\n'panda_description' found!")