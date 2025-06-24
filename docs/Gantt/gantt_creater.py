import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Patch

df = pd.read_csv('tasks.csv')
df = df[df['Task'].notna()]
#df = df.dropna()
df['Start'] = pd.to_datetime(df['Start'], dayfirst=True)
df['End'] = pd.to_datetime(df['End'], dayfirst=True)
# project start date
proj_start = df.Start.min()# number of days from project start to task start
df['start_num'] = (df.Start-proj_start).dt.days# number of days from project start to end of tasks
df['end_num'] = (df.End-proj_start).dt.days# days between start and end of each task
df['days_start_to_end'] = df.end_num - df.start_num

# create a column with the color for each department
def color(row):
    c_dict = {'Holidays':'#E64646', 'System':'#E69646', 'Design':'#34D05C', 'ROS':'#34D0C3', 'Operational':'#3475D0', 'TU/e Contest':'#F8E472' }
    return c_dict.get(row['Category'], '#FFFFFF') 

df['color'] = df.apply(color, axis=1)

df['current_num'] = (df.days_start_to_end *1)# df.Completion)


##### PLOT #####
fig, (ax, ax1) = plt.subplots(2, figsize=(16,6), gridspec_kw={'height_ratios':[6, 1]}, facecolor='#36454F')
fig.set_size_inches(19.2, 10.8)
ax.set_facecolor('#36454F')
ax1.set_facecolor('#36454F')
# bars
mask = df.Category != 'Deadline'
ax.barh(df[mask].Task, df[mask].current_num, left=df[mask].start_num, color=df[mask].color)
ax.barh(df[mask].Task, df[mask].days_start_to_end, left=df[mask].start_num, color=df[mask].color, alpha=0.5)

for idx, row in df.iterrows():
    #ax.text(row.end_num+0.1, idx, f"{int(row.Completion*100)}%", va='center', alpha=0.8, color='w')
    if row['Category'] != "Deadline":
        ax.text(row.start_num-0.1, idx, row.Task, va='center', ha='right', alpha=0.8, color='w')
        #ax.text(row.end_num+0.1, idx, f"FDE = {int(row.Completion*100)}", va='center', alpha=0.8, color='w')
    else:
        ax.axvline(x=row.start_num, color='#E64646', linestyle='--', linewidth=2, alpha=0.8)
        ax.text( row.start_num - 0.9*len(row.Task) , len(df) - len(df[df.Category=='Deadline']) + 1.2, row.Task, rotation=0, color='red', va='center', alpha=1.0, style='oblique',size=13 )#, weight="bold")
    




# grid lines
ax.set_axisbelow(True)
ax.xaxis.grid(color='k', linestyle='dashed', alpha=0.4, which='both')

# ticks

line_spacing = 9
xticks = np.arange(0, df.end_num.max()+1, line_spacing)
xticks_labels = pd.date_range(proj_start, end=df.End.max()).strftime("%d/%m")
xticks_minor = np.arange(0, df.end_num.max()+1, 1)
ax.set_xticks(xticks)
ax.set_xticks(xticks_minor, minor=True)
ax.set_xticklabels(xticks_labels[::line_spacing], color='w')
ax.set_yticks([])

plt.setp([ax.get_xticklines()], color='w')

# align x axis
ax.set_xlim(0, df.end_num.max())

# remove spines
ax.spines['right'].set_visible(False)
ax.spines['left'].set_visible(False)
ax.spines['left'].set_position(('outward', 10))
ax.spines['top'].set_visible(False)
ax.spines['bottom'].set_color('w')



#plt.suptitle('MTMS', color='w', weight= 'bold', size=15)

##### LEGENDS #####
legend_elements = [Patch(facecolor='#E64646', label='Holidays'),
                   Patch(facecolor='#E69646', label='System'),
                   Patch(facecolor='#34D05C', label='Design'),
                   Patch(facecolor='#34D0C3', label='ROS'),
                   Patch(facecolor='#3475D0', label='Operational'),
                   Patch(facecolor='#F8E472',label='TU/e Contest')]


legend = ax1.legend(handles=legend_elements, loc='upper center', ncol=6, frameon=False)
plt.setp(legend.get_texts(), color='w')

# clean second axis
ax1.spines['right'].set_visible(False)
ax1.spines['left'].set_visible(False)
ax1.spines['top'].set_visible(False)
ax1.spines['bottom'].set_visible(False)
ax1.set_xticks([])
ax1.set_yticks([])
plt.show()
#plt.savefig('gantt.png', facecolor='#36454F')

