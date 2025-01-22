import pandas as pd
import matplotlib.pyplot as plt

# Data for the TTC table
data = {
    "Scenario": [
        "C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9", "C10", "C11", "C12", 
        "C13", "C14", "C15", "A1", "A2", "A3", "A4", "A5", "A6", "A7"
    ],
    "TTC": [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 2.8, 3, 2.7, 3.5, 0, 3.8, 4.5, 0, 5.5, 6],
}

# Create a DataFrame
df = pd.DataFrame(data)

# Set the font to Times New Roman and make text bold
plt.rcParams["font.family"] = "Times New Roman"
plt.rcParams["font.weight"] = "bold"

# Plot the data
plt.figure(figsize=(16, 8))

# Define colors for C and A scenarios
colors = ['skyblue' if 'C' in scenario else 'salmon' for scenario in df["Scenario"]]

bars = plt.bar(df["Scenario"], df["TTC"], color=colors, edgecolor="black", linewidth=1.2)

# Add values on top of the bars
for bar in bars:
    height = bar.get_height()
    plt.text(
        bar.get_x() + bar.get_width() / 2, 
        height + 0.2, 
        f'{height:.1f}', 
        ha='center', 
        va='bottom', 
        fontsize=32,
        weight="bold"
    )

# Add chart details
plt.xlabel("Scenario", fontsize=32, weight="bold")
plt.ylabel("TTC", fontsize=32, weight="bold")
plt.xticks(rotation=45, fontsize=32, weight="bold")
plt.yticks(fontsize=32, weight="bold")
plt.grid(axis='y', alpha=0.3)
plt.tight_layout()

# Save the plot
file_path = $your_path$
plt.savefig(file_path)


