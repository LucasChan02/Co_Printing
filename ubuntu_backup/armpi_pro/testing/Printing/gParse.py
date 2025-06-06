import csv
xValOld,yValOld,zValOld=[], [], []
with open ('g_out.csv',mode='r') as file:
    csv_reader=csv.reader(file)

    for row in csv_reader:
        xValOld.append(float(row[0]))
        yValOld.append(float(row[1]))
        zValOld.append(float(row[2]))


xVal = [ (x) for x in xValOld]
yVal = [ (y) for y in yValOld]
zVal = [ (z) for z in zValOld]
