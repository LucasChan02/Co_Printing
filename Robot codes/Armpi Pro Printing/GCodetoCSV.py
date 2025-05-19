import re
import os

try: 
    if os.path.exists('gOut.csv'):
        os.remove('gOut.csv')
except:
    pass

g0pattern = r"^G0.*"
g1pattern = r"^G1.*"
g0re = re.compile(g0pattern)
g1re = re.compile(g1pattern)

Xpattern = r".*X[-\.\d]+"
Ypattern = r".*Y[-\.\d]+"
Zpattern = r".*Z[-\.\d]+"

Xre = re.compile(Xpattern)
Yre = re.compile(Ypattern)
Zre = re.compile(Zpattern)

currentX = 0
currentY = 0
currentZ = 0
fout = open('gOut.csv','w')
should_update = False
with open('tube.g') as f:
    lines = f.readlines()
    for line in lines:
        if g0re.match(line):
            if Zre.match(line):
                should_update = True
                nextZ = re.findall(Zpattern,line)[0].split('Z')[1]
                if currentZ != nextZ:
                    currentZ = nextZ
        elif g1re.match(line):
            print(line)
            if Xre.match(line): 
                should_update = True
                nextX = re.findall(Xpattern,line)[0].split('X')[1]
                if currentX != nextX:
                    currentX = nextX
            if Yre.match(line): 
                should_update = True
                nextY = re.findall(Ypattern,line)[0].split('Y')[1]
                if currentY != nextY:
                    currentY = nextY
            if Zre.match(line):
                should_update = True
                nextZ = re.findall(Zpattern,line)[0].split('Z')[1]
                if currentZ != nextZ:
                    currentZ = nextZ
            if should_update:
                fout.write("{0},{1},{2}\n".format(currentX,currentY,currentZ))
                should_update = False
fout.close()

# **** Not needed ****
# Creates headers x, y, z
# file = pd.read_csv('g_out.csv', header=None)
# file.to_csv('g_out.csv', header=headerList, index=False)