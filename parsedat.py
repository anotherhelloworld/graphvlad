import sys
import os

def parse():
    if len(sys.argv) < 2: 
        print("Please, specify filename")
        return
    filein = sys.argv[1]
    fileout = os.path.splitext(filein)[0] + '.dot'
    
    fin = open(filein, 'r')
    fout = open(fileout, 'w')

    fout.write('graph G {\n')

    for line in fin:
        s = line.split()
        u = int(s[0])
        v = int(s[1])
        w = float(s[2])
        fout.write('\t{} -- {} [label = "{}"]\n'.format(v, u, w))

    fout.write('}\n')

if __name__ == "__main__":
    parse()

