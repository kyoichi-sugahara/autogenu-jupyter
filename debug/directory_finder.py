import os
import time

class DirectoryFinder:
    def __init__(self, directory):
        self.directory = directory

    def find_latest_directory(self, prefix):
        directories = os.listdir(self.directory)
        matched_dirs = [d for d in directories if d.startswith(prefix) and os.path.isdir(os.path.join(self.directory, d))]

        if not matched_dirs:
            raise FileNotFoundError(f"No directories with prefix '{prefix}' found in the specified directory.")

        latest_dir = max(matched_dirs, key=lambda d: int(d.split("_")[-1]))
        latest_dir_path = os.path.join(self.directory, latest_dir)
        return latest_dir_path

    def get_directory_creation_time(self, directory):
        creation_time = os.path.getmtime(directory)
        formatted_time = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(creation_time))
        return formatted_time
