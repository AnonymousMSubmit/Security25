import pandas as pd
import matplotlib.pyplot as plt

# Data for the line chart
data = {
    "Scenario": [
        "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9", "C10",
        "C11", "C12", "C13", "C14", "C15"
    ],
    "10 km/h": [2.5, 3, 2.8, 0, 2.6, 0, 0, 3.1, 2.7, 2.9, 3.3, 2.8, 3.5, 3.6, 3.7],
    "30 km/h": [1.8, 2, 1.9, 0, 1.7, 0, 0, 2.2, 1.6, 2.1, 2.5, 1.9, 2.6, 3.2, 3.3],
    "60 km/h": [0, 0, 0, None, 0, 0, 0, 0, 0, 0, 0, 0, None, None, None],
    "120 km/h": [0, 0, 0, None, 0, None, 0, None, 0, None, 0, None, None, None, None],
}

# Create a DataFrame
df = pd.DataFrame(data)

# Set the font to Times New Roman and make text bold
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.weight"] = "bold"

# Prepare the figure
plt.figure(figsize=(16, 10))

# Define line styles for differentiation
line_styles = ['o-', 's-', '^-', 'd-', 'x-', 'P-', 'h-', 'p-', 'D-', '*-', 'v-', '<-', '>-', '|-', '_-']
colors = plt.cm.tab20.colors  # 20 different colors for lines

# Plot each scenario
for i, scenario in enumerate(df["Scenario"]):
    plt.plot(
        ["10 km/h", "30 km/h", "60 km/h", "120 km/h"],  # X-axis labels
        df.iloc[i, 1:],  # Data for each scenario
        line_styles[i % len(line_styles)],  # Cycle through line styles
        color=colors[i % len(colors)],  # Cycle through colors
        label=scenario,
        markersize=8,
        linewidth=2,
    )

# Add chart details
plt.xlabel("Speed", fontsize=32, weight="bold")
plt.ylabel("TTC", fontsize=32, weight="bold")
plt.xticks(fontsize=32, weight="bold")
plt.yticks(fontsize=32, weight="bold")
plt.legend(fontsize=24, title="Scenario", title_fontsize=28, loc='upper right')
plt.grid(alpha=0.3)
plt.tight_layout()

# Save the plot
file_path = $your_path$
plt.savefig(file_path)








