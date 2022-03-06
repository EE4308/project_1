from operator import delitem
import pandas as pd
import plotly.express as px
import matplotlib.pyplot as plt
import csv

x = []
y = []
with open('data.csv','r') as csvfile:
    plots = csv.reader(csvfile, delimiter=',')
    for row in plots:
        x.append(round(float(row[0]),3))
        y.append(round(float(row[1]),3))
plt.plot(x,y)
plt.gca().set_aspect("equal")
plt.show()

#df = pd.read_csv('data.csv')
#fig = px.line(df, x = 'Y', y = 'X', title='Robot Pos')
#fig.show()
