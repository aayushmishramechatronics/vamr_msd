# this file is a python script to view .pgm files
# cmd to install pillow & matplotlib: `pip install Pillow matplotlib`

import matplotlib.pyplot as plt
from PIL import Image
 
# define the path to your PGM file
file_path = "articubot_map.pgm" # replace with your actual file name

try:
    # open the image using Pillow
    with Image.open(file_path) as img:
        # display the image using matplotlib
        plt.imshow(img, cmap="gray")
        plt.title(f"Viewing: {file_path}")
        plt.axis("off") # Hide grid lines and coordinate axes
        plt.show()

except FileNotFoundError:
    print(f"Error: The file '{file_path}' was not found. Check the path.")
except Exception as e:
    print(f"An error occurred: {e}")

# run the script using: python view_pgm.py
