import numpy as np
import pandas as pd
# Load the .npy file
data = np.load('bajcsy_Brt.npy')

# Print the contents
print(data)

df = pd.DataFrame(data)

# Save the DataFrame to an Excel file
df.to_excel('brt.xlsx', index=False)  #