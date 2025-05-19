import re
import os

def desired_file():
    print('***Default directory is "/home/ubuntu/armpi_pro/testing/Printing/"\n')
    path = input('Please input a path to a directory: ')
    user_choice = str(input('Confirm "' + str(path) + '" is the chosen directory? (y/n)\n').strip().lower())
    if user_choice not in ('y', 'yes'):
        desired_file()

    # /home/ubuntu/armpi_pro/testing/Printing/ should be the default directory if we want to hard code the directory       
    
    file = input('Please specify a desired G-Code file: ')
    if os.path.isfile(path + file):
        user_choice = str(input('Confirm "' + str(file) + '" is the chosen file? (y/n)\n').strip().lower())
        if user_choice not in ('y', 'yes'):
            desired_file()
        return file
    else:
        print('Not a valid file in directory: "' + str(path) + '"\n')
        desired_file()

if __name__ == '__main__':
    try: 
        if os.path.exists('gOut.csv'):
            os.remove('gOut.csv')
    except:
        pass

    fileName = str(desired_file())

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
    with open(fileName) as f:
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