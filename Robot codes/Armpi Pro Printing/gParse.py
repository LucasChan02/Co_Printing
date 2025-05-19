import pandas as pd

data = pd.read_csv('g_out.csv', header=None)
df = pd.DataFrame(data)

xValOld = df[df.columns[0]].tolist()
yValOld = df[df.columns[1]].tolist()
zValOld = df[df.columns[2]].tolist()

xVal = [ (x) for x in xValOld]
yVal = [ (y) for y in yValOld]
zVal = [ (z) for z in zValOld]