import numpy as np
from scipy import io

# Load data from .npy file
data = np.load('reldyn5d_brs_mode0.npy')

# Save data to .mat file
io.savemat('bajcsy_brs.mat', {'valueFunctions': data})
print(data.shape)