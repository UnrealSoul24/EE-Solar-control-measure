import matplotlib.pyplot as plt
import numpy as np

# Define Capital Goods (x-axis)
capital_goods = np.linspace(0, 100, 100)

# Define PPC curves
ppc1 = 100 - 0.01 * (capital_goods ** 2)
ppc2 = 110 - 0.01 * (capital_goods ** 2)
ppc3 = 90 - 0.01 * (capital_goods ** 2)

# Create the plot
plt.figure(figsize=(10, 6))
plt.plot(capital_goods, ppc1, linestyle='-', color='black', label='PPC1')       # Solid line
plt.plot(capital_goods, ppc2, linestyle=':', color='black', label='PPC2')       # Dotted line
plt.plot(capital_goods, ppc3, linestyle='--', color='black', label='PPC3')      # Dashed line

# Arrow properties
arrowprops = dict(facecolor='black', arrowstyle='->', lw=0.8)

# Arrow from PPC1 to PPC2
plt.annotate('', xy=(45, ppc2[45]), xytext=(45, ppc1[45]), arrowprops=arrowprops)

# Arrow from PPC2 to PPC3
plt.annotate('', xy=(75, ppc3[75]), xytext=(75, ppc2[75]), arrowprops=arrowprops)

# Labels and legend
plt.title('Production Possibility Curves (PPC)')
plt.xlabel('Capital Goods (units)')
plt.ylabel('Consumer Goods (units)')
plt.legend()
plt.grid(True)
plt.tight_layout()

# Save as PNG
plt.savefig("PPC_with_arrows.png")
plt.show()
