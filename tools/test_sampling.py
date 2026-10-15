from PIL import Image
import numpy as np

img = Image.open(r"C:\Users\dell\Desktop\rover\tools\tex_0.png")
arr = np.array(img)
h, w, _ = arr.shape

# Let's inspect tools/glb2c/build_final_model.mjs
# Let's check the colors if we sample at vertex corners and pick non-gutter pixels!
