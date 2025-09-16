import argparse
import gdown
import os
import shutil
import pathlib

gdown.download(id="1yxAhYw-ViJnJTKVnvjtYOA78ZXQ_T6io", output="data.7z")
os.system("7z x data.7z")
pathlib.Path("data.7z").unlink()
dir_path = pathlib.Path("data")
if dir_path.exists():
    shutil.rmtree(dir_path)
shutil.move("knapsack_with_conflicts", dir_path)
